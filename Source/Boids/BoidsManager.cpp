#include "BoidsManager.h"
#include "Boid.h"
#include "BoidSystem.h"

ABoidsManager::ABoidsManager()
{
	PrimaryActorTick.bCanEverTick = true;

	NumberOfBoids = 100;

	SpawnVolume = CreateDefaultSubobject<UBoxComponent>(TEXT("SpawnVolume"));
	SpawnVolume->SetupAttachment(RootComponent);
	SpawnVolume->SetBoxExtent(FVector(500.0f, 500.0f, 500.0f));
	SpawnVolume->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	SpawnVolume->SetCollisionResponseToAllChannels(ECR_Ignore);
    
	if (SpawnVolume)
	{
		SpawnVolume->SetHiddenInGame(false);
		SpawnVolume->SetVisibility(true);
	}
}

void ABoidsManager::BeginPlay()
{
	Super::BeginPlay();

	BoidSystem = NewObject<UBoidSystem>(this);
	BoidSystem->OwnerManager = this;

	SyncParametersToSystem();

	SpawnBoids();
}

void ABoidsManager::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	
	if (BoidSystem && !HasAnyFlags(RF_ClassDefaultObject)) // '!HasAnyFlags(RF_ClassDefaultObject)' avoid voodoo error when unreal call this method on the CDO
	{
		SyncParametersToSystem();
	}
}

void ABoidsManager::Tick(const float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidSystem)
	{
		BoidSystem->Update(DeltaTime);
	}
}

void ABoidsManager::SyncParametersToSystem() const
{
	if (BoidSystem)
	{
		BoidSystem->SetBehaviorParameters(
			SeparationWeight,
			AlignmentWeight,
			CohesionWeight,
			SeparationRadius,
			PerceptionRadius,
			BoundaryWeight,
			Velocity,
			FieldOfViewAngle
		);
		
		BoidSystem->SetObstacleAvoidanceParameters(
			ObstacleAvoidanceWeight,
			ObstacleDetectionDistance,
			NumberOfRaycasts,
			bEnableObstacleAvoidance
		);

		BoidSystem->SetUseUniformDistribution(bUseUniformDistribution);
	}
}

void ABoidsManager::RespawnBoids()
{
	if (BoidSystem)
	{
		for (const TArray<ABoid*>& AllActors = BoidSystem->GetActors(); ABoid* Boid : AllActors)
		{
			if (Boid)
			{
				Boid->Destroy();
			}
		}
	}
	SpawnBoids();
}

void ABoidsManager::SetUseUniformDistribution(const bool bNewValue)
{
	bUseUniformDistribution = bNewValue;
    
	if (BoidSystem)
	{
		BoidSystem->SetUseUniformDistribution(bUseUniformDistribution);
	}
}

void ABoidsManager::SpawnBoids()
{
	if (!BoidPrefab)
	{
		UE_LOG(LogTemp, Error, TEXT("Boid non défini dans BoidManager!"));
		return;
	}
	
	const FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	const FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	
	TArray<FVector> InitialPositions;
	TArray<FVector> InitialDirections;
	TArray<ABoid*> SpawnedBoids;
	
	InitialPositions.Reserve(NumberOfBoids);
	InitialDirections.Reserve(NumberOfBoids);
	SpawnedBoids.Reserve(NumberOfBoids);
	
	for (int32 i = 0; i < NumberOfBoids; i++)
	{
		const FVector RandomOffset = FVector(
			FMath::RandRange(-BoxExtent.X, BoxExtent.X),
			FMath::RandRange(-BoxExtent.Y, BoxExtent.Y),
			FMath::RandRange(-BoxExtent.Z, BoxExtent.Z)
		);
        
		const FVector SpawnLocation = BoxOrigin + RandomOffset;
		const FRotator SpawnRotation = FMath::VRand().Rotation();
        
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		if (ABoid* NewBoid = GetWorld()->SpawnActor<ABoid>(BoidPrefab, SpawnLocation, SpawnRotation, SpawnParams))
		{
			NewBoid->BoidsManager = this;
			NewBoid->BoidSystem = BoidSystem;
			
			const FVector RandomDirection = FMath::VRand();
			InitialDirections.Add(RandomDirection);
			
			InitialPositions.Add(SpawnLocation);
			SpawnedBoids.Add(NewBoid);
		}
	}
	
	BoidSystem->Initialize(SpawnedBoids.Num(), InitialPositions, InitialDirections);

	for (int32 i = 0; i < SpawnedBoids.Num(); i++)
	{
		BoidSystem->SetActor(i, SpawnedBoids[i]);
		SpawnedBoids[i]->BoidIndex = i;
	}
}

FVector ABoidsManager::ConstrainPositionToBox(const FVector& Position) const
{
	const FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	const FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	const FVector LocalPos = Position - BoxOrigin;
	
	FVector ConstrainedPos = LocalPos;
    
	if (LocalPos.X < -BoxExtent.X) 
		ConstrainedPos.X = -BoxExtent.X;
	else if (LocalPos.X > BoxExtent.X) 
		ConstrainedPos.X = BoxExtent.X;
        
	if (LocalPos.Y < -BoxExtent.Y) 
		ConstrainedPos.Y = -BoxExtent.Y;
	else if (LocalPos.Y > BoxExtent.Y) 
		ConstrainedPos.Y = BoxExtent.Y;
        
	if (LocalPos.Z < -BoxExtent.Z) 
		ConstrainedPos.Z = -BoxExtent.Z;
	else if (LocalPos.Z > BoxExtent.Z) 
		ConstrainedPos.Z = BoxExtent.Z;
    
	return BoxOrigin + ConstrainedPos;
}

void ABoidsManager::SetNumberOfBoids(const int32 NewValue)
{
	NumberOfBoids = FMath::Clamp(NewValue, 1, 1000);
	SyncParametersToSystem();
}

void ABoidsManager::SetBoidVelocity(const float NewValue)
{
	Velocity = FMath::Clamp(NewValue, 1.0f, 2000.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetPerceptionRadius(const float NewValue)
{
	PerceptionRadius = FMath::Clamp(NewValue, 1.0f, 5000.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetFieldOfViewAngle(const float NewValue)
{
	FieldOfViewAngle = FMath::Clamp(NewValue, 1.0f, 360.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetSeparationWeight(const float NewValue)
{
	SeparationWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetSeparationRadius(const float NewValue)
{
	SeparationRadius = FMath::Clamp(NewValue, 1.0f, 1000.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetAlignmentWeight(const float NewValue)
{
	AlignmentWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetCohesionWeight(const float NewValue)
{
	CohesionWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetEnableObstacleAvoidance(const bool bNewValue)
{
	bEnableObstacleAvoidance = bNewValue;
	SyncParametersToSystem();
}

void ABoidsManager::SetObstacleAvoidanceWeight(const float NewValue)
{
	ObstacleAvoidanceWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetObstacleDetectionDistance(const float NewValue)
{
	ObstacleDetectionDistance = FMath::Clamp(NewValue, 0.1f, 2000.0f);
	SyncParametersToSystem();
}

void ABoidsManager::SetNumberOfRaycasts(const int32 NewValue)
{
	NumberOfRaycasts = FMath::Clamp(NewValue, 1, 12);
	SyncParametersToSystem();

	if (BoidSystem)
	{
		BoidSystem->GenerateRaycastRotators();
	}
}

void ABoidsManager::SetBoundaryWeight(const float NewValue)
{
	BoundaryWeight = FMath::Max(0.0f, NewValue);
	SyncParametersToSystem();
}

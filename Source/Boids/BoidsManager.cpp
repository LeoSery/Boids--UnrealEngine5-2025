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

void ABoidsManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidSystem)
	{
		BoidSystem->Update(DeltaTime);
	}
}

void ABoidsManager::SyncParametersToSystem()
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
			Velocity
		);
	}
}

void ABoidsManager::RespawnBoids()
{
	if (BoidSystem)
	{
		for (ABoid* Boid : BoidSystem->GetActors())
		{
			if (Boid)
			{
				Boid->Destroy();
			}
		}
	}
	
	SpawnBoids();
}

void ABoidsManager::SpawnBoids()
{
	if (!BoidPrefab)
	{
		UE_LOG(LogTemp, Error, TEXT("Boid non défini dans BoidManager!"));
		return;
	}
	
	FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	
	TArray<FVector> InitialPositions;
	TArray<FVector> InitialDirections;
	TArray<ABoid*> SpawnedBoids;
	
	InitialPositions.Reserve(NumberOfBoids);
	InitialDirections.Reserve(NumberOfBoids);
	SpawnedBoids.Reserve(NumberOfBoids);
	
	for (int32 i = 0; i < NumberOfBoids; i++)
	{
		FVector RandomOffset = FVector(
			FMath::RandRange(-BoxExtent.X, BoxExtent.X),
			FMath::RandRange(-BoxExtent.Y, BoxExtent.Y),
			FMath::RandRange(-BoxExtent.Z, BoxExtent.Z)
		);
        
		FVector SpawnLocation = BoxOrigin + RandomOffset;
		FRotator SpawnRotation = FMath::VRand().Rotation();
        
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        
		ABoid* NewBoid = GetWorld()->SpawnActor<ABoid>(BoidPrefab, SpawnLocation, SpawnRotation, SpawnParams);
        
		if (NewBoid)
		{
			FVector RandomDirection = FMath::VRand();
			NewBoid->Direction = RandomDirection;
			NewBoid->BoidsManager = this;
			
			InitialPositions.Add(SpawnLocation);
			InitialDirections.Add(RandomDirection);
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

FVector ABoidsManager::ConstrainPositionToBox(const FVector& Position)
{
	FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	FVector LocalPos = Position - BoxOrigin;
	
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

void ABoidsManager::SetSeparationWeight(float NewValue)
{
	SeparationWeight = FMath::Max(0.0f, NewValue);
	SyncParametersToSystem();
}

void ABoidsManager::SetAlignmentWeight(float NewValue)
{
	AlignmentWeight = FMath::Max(0.0f, NewValue);
	SyncParametersToSystem();
}

void ABoidsManager::SetCohesionWeight(float NewValue)
{
	CohesionWeight = FMath::Max(0.0f, NewValue);
	SyncParametersToSystem();
}

void ABoidsManager::SetSeparationRadius(float NewValue)
{
	SeparationRadius = FMath::Max(1.0f, NewValue);
	SyncParametersToSystem();
}

void ABoidsManager::SetPerceptionRadius(float NewValue)
{
	PerceptionRadius = FMath::Max(1.0f, NewValue);
	SyncParametersToSystem();
}

void ABoidsManager::SetBoidVelocity(float NewValue)
{
	Velocity = FMath::Max(0.1f, NewValue);
	SyncParametersToSystem();
}

void ABoidsManager::SetBoundaryWeight(float NewValue)
{
	BoundaryWeight = FMath::Max(0.0f, NewValue);
	SyncParametersToSystem();
}

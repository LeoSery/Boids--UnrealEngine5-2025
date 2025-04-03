#include "BoidsManager.h"
#include "BoidSystem.h"
#include "Components/InstancedStaticMeshComponent.h"

ABoidsManager::ABoidsManager()
{
	PrimaryActorTick.bCanEverTick = true;

	NumberOfBoids = 100;

	if (!RootComponent)
	{
		RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	}

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

	BoidInstancedMesh = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("BoidInstancedMesh"));
	BoidInstancedMesh->SetupAttachment(RootComponent);
	BoidInstancedMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BoidInstancedMesh->SetGenerateOverlapEvents(false);
	BoidInstancedMesh->SetCastShadow(false);
}

void ABoidsManager::BeginPlay()
{
	Super::BeginPlay();

	BoidSystem = NewObject<UBoidSystem>(this);
	BoidSystem->OwnerManager = this;

	UE_LOG(LogTemp, Warning, TEXT("OwnerManager assigned: %s"), (BoidSystem && BoidSystem->OwnerManager) ? TEXT("Yes") : TEXT("No"));

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
		
		const int32 NumBoids = BoidSystem->GetCount();
		for (int32 i = 0; i < NumBoids; ++i)
		{
			const FVector Position = BoidSystem->GetPosition(i);
			const FVector Direction = BoidSystem->GetDirection(i).GetSafeNormal();
			
			const FRotator DirectionRotation = Direction.Rotation();
			const FRotator FinalRotation = DirectionRotation + MeshRotationOffset;
    
			FTransform InstanceTransform;
			InstanceTransform.SetLocation(Position);
			InstanceTransform.SetRotation(FinalRotation.Quaternion());
			BoidInstancedMesh->UpdateInstanceTransform(i, InstanceTransform, false);
		}
		
		BoidInstancedMesh->MarkRenderStateDirty();
		
		if (bDebugRaycasts)
		{
			const int32 MaxDebugBoids = FMath::Min(5, NumBoids);
			for (int32 i = 0; i < MaxDebugBoids; i++)
			{
				DebugDrawRaycasts(i);
			}
		}
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

void ABoidsManager::SetUseUniformDistribution(const bool bNewValue)
{
	bUseUniformDistribution = bNewValue;
    
	if (BoidSystem)
	{
		BoidSystem->SetUseUniformDistribution(bUseUniformDistribution);
	}
}

void ABoidsManager::DebugDrawRaycasts(const int32 BoidIndex)
{
	if (!BoidSystem || BoidIndex < 0 || BoidIndex >= BoidSystem->GetCount() || !GetWorld())
	{
        return;
	}
        
    const FVector CurrentLocation = BoidSystem->GetPosition(BoidIndex);
    const FVector Direction = BoidSystem->GetDirection(BoidIndex).GetSafeNormal();
    
    DrawDebugLine(
        GetWorld(),
        CurrentLocation,
        CurrentLocation + Direction * 100.0f,
        FColor::Red,
        false,
        0.0f,
        0,
        2.0f
    );
    
    const TArray<FRotator>& RaycastRotators = BoidSystem->GetRaycastRotators();
    const float DetectionDistance = BoidSystem->GetObstacleDetectionDistance();
    
    for (const FRotator& Rotator : RaycastRotators)
    {
        FVector RayDirection = Rotator.RotateVector(Direction);
        
        FHitResult HitResult;
        bool bHit = GetWorld()->LineTraceSingleByChannel(
            HitResult,
            CurrentLocation,
            CurrentLocation + RayDirection * DetectionDistance,
            ECC_WorldStatic,
            FCollisionQueryParams::DefaultQueryParam
        );
        
        FColor LineColor = bHit ? FColor::Green : FColor::Yellow;
        float LineThickness = bHit ? 3.0f : 1.0f;
        
        DrawDebugLine(
            GetWorld(),
            CurrentLocation,
            bHit ? HitResult.ImpactPoint : CurrentLocation + RayDirection * DetectionDistance,
            LineColor,
            false,
            0.0f,
            0,
            LineThickness
        );
        
        if (bHit)
        {
            DrawDebugSphere(
                GetWorld(),
                HitResult.ImpactPoint,
                10.0f,
                8,
                FColor::Red,
                false,
                0.0f,
                0,
                1.0f
            );
            
            DrawDebugLine(
                GetWorld(),
                HitResult.ImpactPoint,
                HitResult.ImpactPoint + HitResult.ImpactNormal * 50.0f,
                FColor::Blue,
                false,
                0.0f,
                0,
                2.0f
            );
        }
    }
}

void ABoidsManager::SpawnBoids() const
{
	if (!BoidMesh)
	{
		UE_LOG(LogTemp, Error, TEXT("Boid not define in BoidManager!"));
		return;
	}

	BoidInstancedMesh->SetStaticMesh(BoidMesh);
	BoidInstancedMesh->ClearInstances();
	
	const FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	const FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	
	TArray<FVector> InitialPositions;
	TArray<FVector> InitialDirections;
	
	InitialPositions.Reserve(NumberOfBoids);
	InitialDirections.Reserve(NumberOfBoids);
	
	for (int32 i = 0; i < NumberOfBoids; i++)
	{
		const FVector RandomOffset = FVector(
			FMath::RandRange(-BoxExtent.X, BoxExtent.X),
			FMath::RandRange(-BoxExtent.Y, BoxExtent.Y),
			FMath::RandRange(-BoxExtent.Z, BoxExtent.Z)
		);
        
		const FVector SpawnLocation = ConstrainPositionToBox(BoxOrigin + RandomOffset);
		const FVector RandomDirection = FMath::VRand();
		const FRotator SpawnRotation = RandomDirection.Rotation();
		
		FTransform InstanceTransform(SpawnRotation, SpawnLocation);
		BoidInstancedMesh->AddInstance(InstanceTransform);
        
		InitialPositions.Add(SpawnLocation);
		InitialDirections.Add(RandomDirection);
	}
	
	BoidSystem->Initialize(NumberOfBoids, InitialPositions, InitialDirections);
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

	if (!LocalPos.Equals(ConstrainedPos, 0.1f))
	{
		UE_LOG(LogTemp, Verbose, TEXT("Position contrainte dans box: %s -> %s"), 
			  *LocalPos.ToString(), *ConstrainedPos.ToString());
	}
    
	return BoxOrigin + ConstrainedPos;
}

void ABoidsManager::SetNumberOfBoids(const int32 NewValue)
{
	NumberOfBoids = FMath::Clamp(NewValue, 1, 2000);
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

void ABoidsManager::SetDebugRaycasts(const bool bNewValue)
{
	if (bDebugRaycasts == bNewValue)
	{
		return;
	}
        
	bDebugRaycasts = bNewValue;
}

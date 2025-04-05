#include "BoidsManager.h"
#include "Boid.h"
#include "BoidSystem.h"

/**
 * @brief Default constructor
 * @details Sets up the SpawnVolume component and initializes base parameters
 */
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

/**
 * @brief Initialization at game start
 * @details Creates the boid system, synchronizes parameters and spawns initial boids
 */
void ABoidsManager::BeginPlay()
{
	Super::BeginPlay();

	BoidSystem = NewObject<UBoidSystem>(this);
	BoidSystem->OwnerManager = this;

	SyncParametersToSystem();

	SpawnBoids();
}

/**
 * @brief Function called every frame
 * @details Updates the boid simulation
 * 
 * @param DeltaTime Time elapsed since the last frame in seconds
 */
void ABoidsManager::Tick(const float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidSystem)
	{
		BoidSystem->Update(DeltaTime);
	}
}

/**
 * @brief Function called during actor construction
 * @details Synchronizes parameters with the boid system
 * 
 * @param Transform The initial transform of the actor
 */
void ABoidsManager::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// Skip for Class Default Object (CDO) - prevents crashes during engine startup
	// and avoids operations on template instances that should not affect the world
	if (BoidSystem && !HasAnyFlags(RF_ClassDefaultObject))
	{
		SyncParametersToSystem();
	}
}

/**
 * @brief Spawns boids within the spawn volume
 * @details Creates the specified number of boids with random positions and directions
 *          within the spawn volume, then initializes the boid system with this data
 */
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
		SpawnedBoids[i]->bDebugRaycasts = bDebugRaycasts;
	}
}

/**
 * @brief Destroys all existing boids and generates new ones
 * @details Useful for resetting the simulation or applying new parameters
 */
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

/**
 * @brief Synchronizes all parameters with the boid system
 * @details Transmits behavior and obstacle avoidance parameters to the system
 */
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

		UE_LOG(LogTemp, Display, TEXT("Boids Collision Parameters Updated:"));
		UE_LOG(LogTemp, Display, TEXT("  - Obstacle Avoidance: %s"), bEnableObstacleAvoidance ? TEXT("Enabled") : TEXT("Disabled"));
		UE_LOG(LogTemp, Display, TEXT("  - Detection Distance: %f"), ObstacleDetectionDistance);
		UE_LOG(LogTemp, Display, TEXT("  - Avoidance Weight: %f"), ObstacleAvoidanceWeight);
		UE_LOG(LogTemp, Display, TEXT("  - Number of Raycasts: %d"), NumberOfRaycasts);
	}
}

/**
 * @brief Constrains a position to remain within the spawn volume boundaries
 * @details Checks each coordinate against the box extents and clamps them if necessary
 * 
 * @param Position The position to constrain
 * @return FVector The constrained position within the boundaries
 */
FVector ABoidsManager::ConstrainPositionToBox(const FVector& Position) const
{
	const FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	const FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	const FVector LocalPos = Position - BoxOrigin;
	
	FVector ConstrainedPos = LocalPos;

	// Clamp position to box boundaries
	// For each axis: if position exceeds box limit, set to the boundary value
	// This creates an invisible barrier at the spawn volume edges to keep boids contained
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

	// Convert back to world space by adding the box origin
	return BoxOrigin + ConstrainedPos;
}

/**
 * @brief Sets the number of boids to generate
 * 
 * @param NewValue The new number of boids (clamped between 1 and 2000)
 */
void ABoidsManager::SetNumberOfBoids(const int32 NewValue)
{
	NumberOfBoids = FMath::Clamp(NewValue, 1, 2000);
	SyncParametersToSystem();
}

/**
 * @brief Sets the movement speed of boids
 * 
 * @param NewValue The new speed (clamped between 1.0 and 2000.0)
 */
void ABoidsManager::SetBoidVelocity(const float NewValue)
{
	Velocity = FMath::Clamp(NewValue, 1.0f, 2000.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the perception radius of boids
 * @details Maximum distance at which a boid can perceive its neighbors
 * 
 * @param NewValue The new perception radius (clamped between 1.0 and 5000.0)
 */
void ABoidsManager::SetPerceptionRadius(const float NewValue)
{
	PerceptionRadius = FMath::Clamp(NewValue, 1.0f, 5000.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the field of view angle for boids
 * @details Determines how wide boids can see around them
 * 
 * @param NewValue The new angle in degrees (clamped between 1.0 and 360.0)
 */
void ABoidsManager::SetFieldOfViewAngle(const float NewValue)
{
	FieldOfViewAngle = FMath::Clamp(NewValue, 1.0f, 360.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the weight of the separation rule
 * @details Influences how strongly boids avoid their close neighbors
 * 
 * @param NewValue The new weight (clamped between 0.1 and 100.0)
 */
void ABoidsManager::SetSeparationWeight(const float NewValue)
{
	SeparationWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the separation radius for boids
 * @details Distance below which boids start to separate from each other
 * 
 * @param NewValue The new radius (clamped between 1.0 and 1000.0)
 */
void ABoidsManager::SetSeparationRadius(const float NewValue)
{
	SeparationRadius = FMath::Clamp(NewValue, 1.0f, 1000.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the weight of the alignment rule
 * @details Influences how strongly boids align with their neighbors' direction
 * 
 * @param NewValue The new weight (clamped between 0.1 and 100.0)
 */
void ABoidsManager::SetAlignmentWeight(const float NewValue)
{
	AlignmentWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the weight of the cohesion rule
 * @details Influences how strongly boids move toward the center of their group
 * 
 * @param NewValue The new weight (clamped between 0.1 and 100.0)
 */
void ABoidsManager::SetCohesionWeight(const float NewValue)
{
	CohesionWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

/**
 * @brief Enables or disables obstacle avoidance
 * 
 * @param bNewValue True to enable, False to disable
 */
void ABoidsManager::SetEnableObstacleAvoidance(const bool bNewValue)
{
	bEnableObstacleAvoidance = bNewValue;
	SyncParametersToSystem();
}

/**
 * @brief Sets the weight of obstacle avoidance
 * @details Influences how strongly boids steer away from detected obstacles
 * 
 * @param NewValue The new weight (clamped between 0.1 and 100.0)
 */
void ABoidsManager::SetObstacleAvoidanceWeight(const float NewValue)
{
	ObstacleAvoidanceWeight = FMath::Clamp(NewValue, 0.1f, 100.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the obstacle detection distance
 * @details Maximum distance at which a boid can detect obstacles
 * 
 * @param NewValue The new distance (clamped between 0.1 and 2000.0)
 */
void ABoidsManager::SetObstacleDetectionDistance(const float NewValue)
{
	ObstacleDetectionDistance = FMath::Clamp(NewValue, 0.1f, 2000.0f);
	SyncParametersToSystem();
}

/**
 * @brief Sets the number of rays used for obstacle detection
 * @details More rays = more precise detection but higher performance cost
 * 
 * @param NewValue The new number of rays (clamped between 1 and 12)
 */
void ABoidsManager::SetNumberOfRaycasts(const int32 NewValue)
{
	NumberOfRaycasts = FMath::Clamp(NewValue, 1, 12);
	SyncParametersToSystem();

	if (BoidSystem)
	{
		BoidSystem->GenerateRaycastRotators();
	}
}

/**
 * @brief Sets whether obstacle detection rays are uniformly distributed
 * @details True: uniform distribution on a sphere (better coverage)
 *          False: predefined distribution (more predictable)
 * 
 * @param bNewValue The new distribution state
 */
void ABoidsManager::SetUseUniformDistribution(const bool bNewValue)
{
	bUseUniformDistribution = bNewValue;
    
	if (BoidSystem)
	{
		BoidSystem->SetUseUniformDistribution(bUseUniformDistribution);
	}
}
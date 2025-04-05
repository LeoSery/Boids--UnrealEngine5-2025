#include "Boid.h"
#include "BoidSystem.h"

/**
 * @brief Default constructor
 * @details Sets up the basic components hierarchy with Root and Mesh,
 *          and configures physics and collision properties
 */
ABoid::ABoid()
{
	PrimaryActorTick.bCanEverTick = true;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;
	
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	Mesh->SetupAttachment(Root);
	
	Mesh->SetSimulatePhysics(false);
	Mesh->SetEnableGravity(false);
	Mesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
}

/**
 * @brief Initialization at game start
 * @details Sets the initial position and rotation based on data from the BoidSystem
 */
void ABoid::BeginPlay()
{
	Super::BeginPlay();
    
	if (BoidSystem && BoidIndex >= 0)
	{
		SetActorLocationAndRotation(
			BoidSystem->GetPosition(BoidIndex),
			BoidSystem->GetDirection(BoidIndex).Rotation()
		);
	}
}

/**
 * @brief Function called every frame
 * @details Updates the actor's position and rotation to match the simulation data,
 *          and optionally draws debug visualizations
 * 
 * @param DeltaTime Time elapsed since the last frame in seconds
 */
void ABoid::Tick(const float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!BoidSystem || BoidIndex < 0)
	{
		return;
	}

	const FVector NewPosition = BoidSystem->GetPosition(BoidIndex);
	const FVector Direction = BoidSystem->GetDirection(BoidIndex).GetSafeNormal();

	if (constexpr float MinUpdateDistanceSquared = 1.0f; FVector::DistSquared(GetActorLocation(), NewPosition) > MinUpdateDistanceSquared)
	{
		SetActorLocationAndRotation(NewPosition, Direction.Rotation());
	}
	
	if (bDebugRaycasts)
	{
		DebugDrawRaycasts();
	}
}

/**
 * @brief Draws debug visualization for obstacle detection raycasts
 * @details Visualizes the boid's forward direction and all obstacle detection rays,
 *          showing hits with spheres and impact normals
 * 
 * Colors:
 * - Red: Boid's forward direction
 * - Yellow: Rays with no hits
 * - Green: Rays with hits (thicker)
 * - Red Sphere: Hit impact point
 * - Blue: Hit normal direction
 */
void ABoid::DebugDrawRaycasts() const
{
    if (!BoidSystem || BoidIndex < 0 || !GetWorld())
    {
        return;
    }
        
    const FVector CurrentLocation = GetActorLocation();
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
        const bool bHit = GetWorld()->LineTraceSingleByChannel(
            HitResult,
            CurrentLocation,
            CurrentLocation + RayDirection * DetectionDistance,
            ECC_WorldStatic,
            FCollisionQueryParams::DefaultQueryParam
        );
        
        FColor LineColor = bHit ? FColor::Green : FColor::Yellow;
        const float LineThickness = bHit ? 3.0f : 1.0f;
        
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

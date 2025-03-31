#include "Boid.h"

#include "BoidsManager.h"

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

void ABoid::DebugDrawRaycasts()
{
    if (!BoidSystem || BoidIndex < 0 || !GetWorld())
        return;
        
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

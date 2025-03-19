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
	
	Velocity = 1.0f;
	Direction = FVector(1.0f, 0.0f, 0.0f);
}

void ABoid::BeginPlay()
{
	Super::BeginPlay();

	FOVDotProductThreshold = FMath::Cos(FMath::DegreesToRadians(FieldOfViewAngle * 0.5f));
}

FVector ABoid::ComputeSeparation(const TArray<ABoid*>& NearbyBoids)
{
	FVector SeparationForce = FVector::ZeroVector;
    
	if (NearbyBoids.Num() == 0)
	{
		return SeparationForce;
	}
    
	FVector MyLocation = GetActorLocation();
	int32 BoidsCount = 0;
    
	for (ABoid* OtherBoid : NearbyBoids)
	{
		FVector OtherLocation = OtherBoid->GetActorLocation();
		float Distance = FVector::Dist(MyLocation, OtherLocation);
		
		if (Distance < SeparationRadius && Distance > 0)
		{
			FVector AwayFromOther = MyLocation - OtherLocation;
			AwayFromOther.Normalize();

			float StrengthFactor = FMath::Square(SeparationRadius / FMath::Max(Distance, 1.0f));
			AwayFromOther = AwayFromOther * StrengthFactor;
            
			SeparationForce += AwayFromOther;
			BoidsCount++;
		}
	}
	
	if (BoidsCount > 0)
	{
		SeparationForce = SeparationForce / BoidsCount;
        
		if (!SeparationForce.IsNearlyZero())
		{
			SeparationForce.Normalize();
		}
	}
    
	return SeparationForce;
}

FVector ABoid::ComputeAlignment(const TArray<ABoid*>& NearbyBoids)
{
	if (NearbyBoids.Num() == 0)
	{
		return FVector::ZeroVector;
	}
	
	FVector AverageDirection = FVector::ZeroVector;
    
	for (ABoid* OtherBoid : NearbyBoids)
	{
		AverageDirection += OtherBoid->Direction;
	}

	AverageDirection = AverageDirection / NearbyBoids.Num();
	
	if (!AverageDirection.IsNearlyZero())
	{
		AverageDirection.Normalize();
	}
	
	return AverageDirection;
}

FVector ABoid::ComputeCohesion(const TArray<ABoid*>& NearbyBoids)
{
	if (NearbyBoids.Num() == 0)
	{
		return FVector::ZeroVector;
	}
	
	FVector CenterOfMass = FVector::ZeroVector;
    
	for (ABoid* OtherBoid : NearbyBoids)
	{
		CenterOfMass += OtherBoid->GetActorLocation();
	}
	
	CenterOfMass = CenterOfMass / NearbyBoids.Num();
	
	FVector MyLocation = GetActorLocation();
	FVector DirectionToCenter = CenterOfMass - MyLocation;
	
	if (!DirectionToCenter.IsNearlyZero())
	{
		DirectionToCenter.Normalize();
	}
    
	return DirectionToCenter;
}

FVector ABoid::ComputeObstacleAvoidance()
{
	if (!bEnableObstacleAvoidance)
    {
        return FVector::ZeroVector;
    }

    FVector AvoidanceForce = FVector::ZeroVector;
    FVector MyLocation = GetActorLocation();
    int32 HitCount = 0;
	
    TArray<FVector> Directions;
    Directions.Add(Direction.GetSafeNormal());
    Directions.Add(FRotator(0, 30, 0).RotateVector(Direction).GetSafeNormal());  // +30° yaw
    Directions.Add(FRotator(0, -30, 0).RotateVector(Direction).GetSafeNormal()); // -30° yaw
    Directions.Add(FRotator(30, 0, 0).RotateVector(Direction).GetSafeNormal());  // +30° pitch
    Directions.Add(FRotator(-30, 0, 0).RotateVector(Direction).GetSafeNormal()); // -30° pitch
	
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this);
	
    if (BoidsManager)
    {
        for (ABoid* OtherBoid : BoidsManager->GetAllBoids())
        {
            if (OtherBoid != this)
            {
                QueryParams.AddIgnoredActor(OtherBoid);
            }
        }
    }
    
    for (FVector Dir : Directions)
    {
        FHitResult HitResult;
    	
        bool bHit = GetWorld()->LineTraceSingleByChannel(
            HitResult,
            MyLocation,
            MyLocation + Dir * ObstacleDetectionDistance,
            ECC_WorldStatic,
            QueryParams
        );
        
        if (bHit)
        {
            float Distance = HitResult.Distance;
            float StrengthFactor = 1.0f - (Distance / ObstacleDetectionDistance);
        	
            FVector AwayFromObstacle = -Dir * StrengthFactor * 2.0f;
            AvoidanceForce += AwayFromObstacle;
            HitCount++;
        }
    }
    
    if (HitCount > 0)
    {
        AvoidanceForce = AvoidanceForce / HitCount;
        if (!AvoidanceForce.IsNearlyZero())
        {
            AvoidanceForce.Normalize();
        }
    }
    
    return AvoidanceForce;
}

FVector ABoid::ComputeBoundaryForce()
{
	if (!BoidsManager)
        return FVector::ZeroVector;
        
    FVector BoundaryForce = FVector::ZeroVector;
    FVector MyLocation = GetActorLocation();
	
    FVector BoxOrigin = BoidsManager->SpawnVolume->GetComponentLocation();
    FVector BoxExtent = BoidsManager->SpawnVolume->GetScaledBoxExtent();
	
    FVector LocalPos = MyLocation - BoxOrigin;
	
    const float BoundaryMargin = 50.0f;
    const float ForceStrength = 1.0f;
	
    if (LocalPos.X > BoxExtent.X - BoundaryMargin)
        BoundaryForce.X = -ForceStrength * (1.0f - ((BoxExtent.X - LocalPos.X) / BoundaryMargin));
    else if (LocalPos.X < -BoxExtent.X + BoundaryMargin)
        BoundaryForce.X = ForceStrength * (1.0f - ((-BoxExtent.X - LocalPos.X) / -BoundaryMargin));
        
    if (LocalPos.Y > BoxExtent.Y - BoundaryMargin)
        BoundaryForce.Y = -ForceStrength * (1.0f - ((BoxExtent.Y - LocalPos.Y) / BoundaryMargin));
    else if (LocalPos.Y < -BoxExtent.Y + BoundaryMargin)
        BoundaryForce.Y = ForceStrength * (1.0f - ((-BoxExtent.Y - LocalPos.Y) / -BoundaryMargin));
        
    if (LocalPos.Z > BoxExtent.Z - BoundaryMargin)
        BoundaryForce.Z = -ForceStrength * (1.0f - ((BoxExtent.Z - LocalPos.Z) / BoundaryMargin));
    else if (LocalPos.Z < -BoxExtent.Z + BoundaryMargin)
        BoundaryForce.Z = ForceStrength * (1.0f - ((-BoxExtent.Z - LocalPos.Z) / -BoundaryMargin));
	
    if (!BoundaryForce.IsNearlyZero())
    {
        DrawDebugLine(
            GetWorld(),
            MyLocation,
            MyLocation + BoundaryForce * 100.0f,
            FColor::Yellow,
            false,
            -1.0f,
            0,
            2.0f
        );
    }
    
    return BoundaryForce;
}

void ABoid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidsManager)
	{
		TArray<ABoid*> NearbyBoids = BoidsManager->GetNearbyBoids(this, PerceptionRadius);
		
		FVector AlignmentForce = ComputeAlignment(NearbyBoids);
		FVector SeparationForce = ComputeSeparation(NearbyBoids);
		FVector CohesionForce = ComputeCohesion(NearbyBoids);
		FVector ObstacleAvoidanceForce = ComputeObstacleAvoidance();
		FVector BoundaryForce = ComputeBoundaryForce();

		FVector TargetDirection = AlignmentForce  * AlignmentWeight
					   + SeparationForce * SeparationWeight
					   + CohesionForce   * CohesionWeight
					   + ObstacleAvoidanceForce * ObstacleAvoidanceWeight
					   + BoundaryForce *  BoundraryWeight;

		if (TargetDirection.IsNearlyZero())
		{
			TargetDirection = Direction.GetSafeNormal();
		}
		else
		{
			TargetDirection.Normalize();
		}
		
		Direction = FMath::VInterpNormalRotationTo(
			Direction,
			TargetDirection,
			DeltaTime,
			90.0f 
		);
	}

	FVector CurrentLocation = GetActorLocation();
	FVector NewLocation = CurrentLocation + (Direction * Velocity * DeltaTime);

	if (BoidsManager)
	{
		NewLocation = BoidsManager->ConstrainPositionToBox(NewLocation);
	}
	
	SetActorLocation(NewLocation);

	FRotator NewRotation = Direction.Rotation();
	SetActorRotation(NewRotation);
	
	FVector ForwardVector = GetActorForwardVector();
	FVector UpVector = GetActorUpVector();
	
	const float LineLength = 100.0f;

	// Direction (red)
	DrawDebugLine(
		GetWorld(),
		CurrentLocation,
		CurrentLocation + Direction.GetSafeNormal() * LineLength,
		FColor::Red,
		false,
		-1.0f,
		0,
		2.0f
	);
    
	// Forward Vector (blue)
	DrawDebugLine(
		GetWorld(),
		CurrentLocation,
		CurrentLocation + ForwardVector * LineLength,
		FColor::Blue,
		false,
		-1.0f,
		0,
		2.0f
	);
    
	// Up Vector (green)
	DrawDebugLine(
		GetWorld(),
		CurrentLocation,
		CurrentLocation + UpVector * LineLength,
		FColor::Green,
		false,
		-1.0f,
		0,
		2.0f
	);

	FHitResult TestHit;
	bool bHits = GetWorld()->LineTraceSingleByChannel(
		TestHit,
		GetActorLocation(),
		GetActorLocation() + FVector(0, 0, 5000),
		ECC_Visibility,
		FCollisionQueryParams()
	);

	if (bHits)
	{
		UE_LOG(LogTemp, Warning, TEXT("Hit something: %s"), *TestHit.GetActor()->GetName());
	}
}

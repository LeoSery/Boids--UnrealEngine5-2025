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
			
			//AwayFromOther = AwayFromOther * (SeparationRadius / Distance);

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

void ABoid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BoidsManager)
	{
		TArray<ABoid*> NearbyBoids = BoidsManager->GetNearbyBoids(this, PerceptionRadius);
		
		FVector AlignmentForce = ComputeAlignment(NearbyBoids);
		FVector SeparationForce = ComputeSeparation(NearbyBoids);
		FVector CohesionForce = ComputeCohesion(NearbyBoids);

		FVector NewDirection = Direction;
		
		if (!AlignmentForce.IsNearlyZero())
		{
			NewDirection += AlignmentForce * AlignmentWeight;
			NewDirection.Normalize();
		}

		if (!SeparationForce.IsNearlyZero())
		{
			NewDirection += SeparationForce * SeparationWeight;
			NewDirection.Normalize();
		}

		if (!CohesionForce.IsNearlyZero())
		{
			NewDirection += CohesionForce * CohesionWeight;
			NewDirection.Normalize();
		}

		if (!NewDirection.IsNearlyZero())
		{
			//Direction = NewDirection;
			Direction = FMath::VInterpNormalRotationTo(
				Direction,             // Direction actuelle
				NewDirection,          // Direction cible
				DeltaTime * 5.0f,      // Facteur d'interpolation (ajuster selon besoin)
				0.0f                   // Tolérance
			);
		}
	}

	FVector CurrentLocation = GetActorLocation();
	FVector NewLocation = CurrentLocation + Direction * Velocity * DeltaTime;

	if (BoidsManager)
	{
		NewLocation = BoidsManager->ConstrainPositionToBox(NewLocation);
		
		FVector BoxOrigin = BoidsManager->SpawnVolume->GetComponentLocation();
		FVector BoxExtent = BoidsManager->SpawnVolume->GetScaledBoxExtent();
		FVector LocalPos = NewLocation - BoxOrigin;
		
		if (FMath::Abs(FMath::Abs(LocalPos.X) - BoxExtent.X) < 5.0f)
			Direction.X *= -1.0f;
		if (FMath::Abs(FMath::Abs(LocalPos.Y) - BoxExtent.Y) < 5.0f)
			Direction.Y *= -1.0f;
		if (FMath::Abs(FMath::Abs(LocalPos.Z) - BoxExtent.Z) < 5.0f)
			Direction.Z *= -1.0f;
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
}

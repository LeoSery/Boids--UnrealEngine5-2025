#include "BoidsManager.h"

#include "Boid.h"

ABoidsManager::ABoidsManager()
{
	PrimaryActorTick.bCanEverTick = true;

	NumberOfBoids = 10;
	SpawnRadius = 500.0f;
}

void ABoidsManager::SpawnBoids()
{
	Boids.Empty();
	
	if (!BoidPrefab)
	{
		UE_LOG(LogTemp, Error, TEXT("Boid non défini dans BoidManager!"));
		return;
	}
	
	FVector BaseLocation = GetActorLocation();
	
	for (int32 i = 0; i < NumberOfBoids; i++)
	{
		FVector RandomOffset = FMath::VRand() * FMath::RandRange(0.0f, SpawnRadius);
		FVector SpawnLocation = BaseLocation + RandomOffset;
		FRotator SpawnRotation = FMath::VRand().Rotation();
		
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		
		ABoid* NewBoid = GetWorld()->SpawnActor<ABoid>(BoidPrefab, SpawnLocation, SpawnRotation, SpawnParams);
        
		if (NewBoid)
		{
			NewBoid->Direction = FMath::VRand();
			Boids.Add(NewBoid);
		}
	}
}

TArray<ABoid*> ABoidsManager::GetNearbyBoids(ABoid* Boid, float Radius) const
{
	TArray<ABoid*> NearbyBoids;
    
	if (!Boid)
	{
		return NearbyBoids;
	}
    
	FVector BoidLocation = Boid->GetActorLocation();
	float RadiusSquared = Radius * Radius;
	
	for (ABoid* OtherBoid : Boids)
	{
		if (OtherBoid == Boid)
		{
			continue;
		}
		
		float DistanceSquared = FVector::DistSquared(BoidLocation, OtherBoid->GetActorLocation());
		
		if (DistanceSquared <= RadiusSquared)
		{
			NearbyBoids.Add(OtherBoid);
		}
	}
    
	return NearbyBoids;
}

void ABoidsManager::BeginPlay()
{
	Super::BeginPlay();
	SpawnBoids();
}

void ABoidsManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

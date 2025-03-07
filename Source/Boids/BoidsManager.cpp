#include "BoidsManager.h"

#include "Boid.h"

ABoidsManager::ABoidsManager()
{
	PrimaryActorTick.bCanEverTick = true;

	NumberOfBoids = 10;

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
	SpawnBoids();
}

void ABoidsManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


void ABoidsManager::SpawnBoids()
{
	Boids.Empty();
	
	if (!BoidPrefab)
	{
		UE_LOG(LogTemp, Error, TEXT("Boid non défini dans BoidManager!"));
		return;
	}
	
	FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	
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
			NewBoid->Direction = FMath::VRand();
			NewBoid->BoidsManager = this;
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

FVector ABoidsManager::ConstrainPositionToBox(const FVector& Position)
{
	FVector BoxOrigin = SpawnVolume->GetComponentLocation();
	FVector BoxExtent = SpawnVolume->GetScaledBoxExtent();
	FVector LocalPos = Position - BoxOrigin;
    
	// Contraindre les positions
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

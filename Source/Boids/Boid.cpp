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

void ABoid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!BoidSystem || BoidIndex < 0)
	{
		return;
	}

	FVector Direction = BoidSystem->GetDirection(BoidIndex).GetSafeNormal();
	FVector CurrentLocation = GetActorLocation();
	FVector ForwardVector = GetActorForwardVector();
	FVector UpVector = GetActorUpVector();
	
	const float LineLength = 100.0f;

	// Direction (red)
	DrawDebugLine(
		GetWorld(),
		CurrentLocation,
		CurrentLocation + Direction * LineLength,
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

	// FHitResult TestHit;
	// bool bHits = GetWorld()->LineTraceSingleByChannel(
	// 	TestHit,
	// 	GetActorLocation(),
	// 	GetActorLocation() + FVector(0, 0, 5000),
	// 	ECC_Visibility,
	// 	FCollisionQueryParams()
	// );
	//
	// if (bHits)
	// {
	// 	UE_LOG(LogTemp, Warning, TEXT("Hit something: %s"), *TestHit.GetActor()->GetName());
	// }
}

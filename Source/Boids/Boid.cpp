#include "Boid.h"

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

void ABoid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	FVector CurrentLocation = GetActorLocation();
	FVector NewLocation = CurrentLocation + Direction * Velocity * DeltaTime;
	SetActorLocation(NewLocation);

	FRotator NewRotation = Direction.Rotation();
    
	SetActorRotation(NewRotation);
	
	FVector ForwardVector = GetActorForwardVector();
	FVector UpVector = GetActorUpVector();
    
	// Longueur des lignes de debug
	const float LineLength = 100.0f;
    
	// Dessiner Direction (rouge)
	DrawDebugLine(
		GetWorld(),
		CurrentLocation,
		CurrentLocation + Direction.GetSafeNormal() * LineLength,
		FColor::Red,
		false, // Persistant
		-1.0f, // Durée (négatif = une frame)
		0,     // Priorité
		2.0f   // Épaisseur
	);
    
	// Dessiner Forward Vector (bleu)
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
    
	// Dessiner Up Vector (vert)
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

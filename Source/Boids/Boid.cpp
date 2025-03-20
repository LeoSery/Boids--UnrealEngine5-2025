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
	
	Direction = FVector(1.0f, 0.0f, 0.0f);
}

void ABoid::BeginPlay()
{
	Super::BeginPlay();

	GenerateRaycastRotators();
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
    
	for (const FRotator& Rotator : RaycastRotators)
	{
		FVector WorldDir = Rotator.RotateVector(Direction.GetSafeNormal());
        
		FHitResult HitResult;
        
		bool bHit = GetWorld()->LineTraceSingleByChannel(
			HitResult,
			MyLocation,
			MyLocation + WorldDir * ObstacleDetectionDistance,
			ECC_WorldStatic,
			QueryParams
		);
		
		// DrawDebugLine(GetWorld(), MyLocation, MyLocation + WorldDir * ObstacleDetectionDistance, 
		//               bHit ? FColor::Red : FColor::Green, false, -1.0f, 0, 1.0f);
        
		if (bHit)
		{
			float Distance = HitResult.Distance;
			float StrengthFactor = 1.0f - (Distance / ObstacleDetectionDistance);
            
			FVector AwayFromObstacle = -WorldDir * StrengthFactor * 2.0f;
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

void ABoid::GenerateRaycastRotators()
{
	RaycastRotators.Empty();
	
	RaycastRotators.Add(FRotator::ZeroRotator);
	
	float YawAngle = 30.0f;    // Angle horizontal
	float PitchAngle = 30.0f;  // Angle vertical
	
	RaycastRotators.Add(FRotator(0, YawAngle, 0));       // Droite
	RaycastRotators.Add(FRotator(0, -YawAngle, 0));      // Gauche
	RaycastRotators.Add(FRotator(PitchAngle, 0, 0));     // Haut
	RaycastRotators.Add(FRotator(-PitchAngle, 0, 0));    // Bas
	
	if (NumberOfRaycasts > 5)
	{
		RaycastRotators.Add(FRotator(PitchAngle, YawAngle, 0));     // Haut-Droite
		RaycastRotators.Add(FRotator(PitchAngle, -YawAngle, 0));    // Haut-Gauche
		RaycastRotators.Add(FRotator(-PitchAngle, YawAngle, 0));    // Bas-Droite
		RaycastRotators.Add(FRotator(-PitchAngle, -YawAngle, 0));   // Bas-Gauche
	}
	
	if (NumberOfRaycasts > 9)
	{
		float HalfYaw = YawAngle * 0.5f;
		float HalfPitch = PitchAngle * 0.5f;
        
		RaycastRotators.Add(FRotator(0, HalfYaw, 0));              // Demi-droite
		RaycastRotators.Add(FRotator(0, -HalfYaw, 0));             // Demi-gauche
		RaycastRotators.Add(FRotator(HalfPitch, 0, 0));            // Demi-haut
		RaycastRotators.Add(FRotator(-HalfPitch, 0, 0));           // Demi-bas
	}
	
	while (RaycastRotators.Num() > NumberOfRaycasts)
	{
		RaycastRotators.RemoveAt(RaycastRotators.Num() - 1);
	}
}

void ABoid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	FVector CurrentLocation = GetActorLocation();
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

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Boid.generated.h"

class ABoidsManager;
class UBoidSystem;

UCLASS()
class BOIDS_API ABoid : public AActor
{
	GENERATED_BODY()

public:
	ABoid();
	
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boids|Settings")
	UStaticMeshComponent* Mesh;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	bool bEnableObstacleAvoidance = true;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	float ObstacleDetectionDistance = 300.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	float ObstacleAvoidanceWeight = 2.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	int32 NumberOfRaycasts = 8;

	FVector Direction;
	
	UPROPERTY()
	ABoidsManager* BoidsManager;

	int32 BoidIndex = -1;

	UPROPERTY()
	UBoidSystem* BoidSystem = nullptr;
	
	FVector ComputeObstacleAvoidance();
	
protected:
	virtual void BeginPlay() override;
	
	void GenerateRaycastRotators();

	UPROPERTY()
	TArray<FRotator> RaycastRotators;

	UPROPERTY()
	USceneComponent* Root;
};

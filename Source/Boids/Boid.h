#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Boid.generated.h"

class ABoidsManager;

UCLASS()
class BOIDS_API ABoid : public AActor
{
	GENERATED_BODY()

public:
	ABoid();
	
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, Category = "Boids|Settings")
	float Velocity;

	UPROPERTY()
	FVector Direction;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boids|Settings")
	UStaticMeshComponent* Mesh;

	UPROPERTY(EditAnywhere, Category = "Boids|Behavior")
	float PerceptionRadius = 200.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Behavior") 
	float SeparationWeight = 1.5f;

	UPROPERTY(EditAnywhere, Category = "Boids|Behavior")
	float AlignmentWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Behavior")
	float CohesionWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Behavior")
	float SeparationRadius = 100.0f;
	
	UPROPERTY(EditAnywhere, Category = "Boids|Perception")
	float FieldOfViewAngle = 120.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	bool bEnableObstacleAvoidance = true;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	float ObstacleDetectionDistance = 300.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	float ObstacleAvoidanceWeight = 2.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Behavior")
	float BoundraryWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
	int32 NumberOfRaycasts = 8;

	FORCEINLINE float GetFOVDotProductThreshold() const { return FOVDotProductThreshold; }
	
	UPROPERTY()
	ABoidsManager* BoidsManager;
	
protected:
	virtual void BeginPlay() override;

	FVector ComputeSeparation(const TArray<ABoid*>& NearbyBoids);
	FVector ComputeAlignment(const TArray<ABoid*>& NearbyBoids);
	FVector ComputeCohesion(const TArray<ABoid*>& NearbyBoids);
	FVector ComputeObstacleAvoidance();
	FVector ComputeBoundaryForce();

	float FOVDotProductThreshold = 0.5f;

	UPROPERTY()
	TArray<FRotator> RaycastRotators;

	void GenerateRaycastRotators();

	UPROPERTY()
	USceneComponent* Root;
};

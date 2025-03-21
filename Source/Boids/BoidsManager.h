#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "BoidSystem.h"
#include "BoidsManager.generated.h"

class ABoid;

UCLASS()
class BOIDS_API ABoidsManager : public AActor
{
    GENERATED_BODY()

public:
    ABoidsManager();
    virtual void Tick(float DeltaTime) override;
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    
    void SpawnBoids();
    FVector ConstrainPositionToBox(const FVector& Position);
    
    void SyncParametersToSystem();
    
    FORCEINLINE int32 GetNumberOfBoids() const { return NumberOfBoids; }
    FORCEINLINE float GetSeparationWeight() const { return SeparationWeight; }
    FORCEINLINE float GetAlignmentWeight() const { return AlignmentWeight; }
    FORCEINLINE float GetCohesionWeight() const { return CohesionWeight; }
    FORCEINLINE float GetSeparationRadius() const { return SeparationRadius; }
    FORCEINLINE float GetPerceptionRadius() const { return PerceptionRadius; }
    FORCEINLINE float GetBoidVelocity() const { return Velocity; }
    FORCEINLINE float GetBoundaryWeight() const { return BoundaryWeight; }
    FORCEINLINE float GetFieldOfViewAngle() const { return FieldOfViewAngle; }
    FORCEINLINE float GetObstacleAvoidanceWeight() const { return ObstacleAvoidanceWeight; }
    FORCEINLINE bool GetEnableObstacleAvoidance() const { return bEnableObstacleAvoidance; }
    FORCEINLINE float GetObstacleDetectionDistance() const { return ObstacleDetectionDistance; }
    FORCEINLINE int32 GetNumberOfRaycasts() const { return NumberOfRaycasts; }
    FORCEINLINE TArray<ABoid*> GetAllBoids() const { return BoidSystem ? BoidSystem->GetActors() : TArray<ABoid*>(); }
    
    UFUNCTION(Category = "Boids|Spawning")
    FORCEINLINE void SetNumberOfBoids(int32 NewValue)
    {
        NumberOfBoids = FMath::Max(1, NewValue);
    }
    
    UFUNCTION(Category = "Boids|Behavior")
    void SetSeparationWeight(float NewValue);
    
    UFUNCTION(Category = "Boids|Behavior")
    void SetAlignmentWeight(float NewValue);
    
    UFUNCTION(Category = "Boids|Behavior")
    void SetCohesionWeight(float NewValue);
    
    UFUNCTION(Category = "Boids|Behavior")
    void SetSeparationRadius(float NewValue);
    
    UFUNCTION(Category = "Boids|Behavior")
    void SetPerceptionRadius(float NewValue);

    UFUNCTION(Category = "Boids|Movement")
    void SetBoidVelocity(float NewValue);

    UFUNCTION(Category = "Boids|Behavior")
    void SetBoundaryWeight(float NewValue);

    UFUNCTION(Category = "Boids|Perception")
    void SetFieldOfViewAngle(float NewValue);

    UFUNCTION(Category = "Boids|Obstacle Avoidance")
    void SetObstacleAvoidanceWeight(float NewValue);

    UFUNCTION(Category = "Boids|Obstacle Avoidance")
    void SetEnableObstacleAvoidance(bool bNewValue);

    UFUNCTION(Category = "Boids|Obstacle Avoidance")
    void SetObstacleDetectionDistance(float NewValue);

    UFUNCTION(Category = "Boids|Obstacle Avoidance")
    void SetNumberOfRaycasts(int32 NewValue);
    
    UFUNCTION(Category = "Boids|Spawning")
    void RespawnBoids();
    
    UPROPERTY(EditAnywhere, Category = "Boids|Spawning")
    TSubclassOf<ABoid> BoidPrefab;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Boids|Spawning")
    UBoxComponent* SpawnVolume;

protected:
    UPROPERTY()
    UBoidSystem* BoidSystem;

private:
    UPROPERTY(EditAnywhere, Category = "Boids|Spawning", meta = (ClampMin = "1", ClampMax = "10000"))
    int32 NumberOfBoids = 100;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior", meta = (ClampMin = "0.0"))
    float SeparationWeight = 1.5f;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior", meta = (ClampMin = "0.0"))
    float AlignmentWeight = 1.0f;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior", meta = (ClampMin = "0.0"))
    float CohesionWeight = 1.0f;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior", meta = (ClampMin = "1.0"))
    float SeparationRadius = 100.0f;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior", meta = (ClampMin = "1.0"))
    float PerceptionRadius = 200.0f;

    UPROPERTY(EditAnywhere, Category = "Boids|Movement", meta = (ClampMin = "0.1"))
    float Velocity = 1000.0f;

    UPROPERTY(EditAnywhere, Category = "Boids|Behavior", meta = (ClampMin = "0.0"))
    float BoundaryWeight = 1.0f;

    UPROPERTY(EditAnywhere, Category = "Boids|Perception", meta = (ClampMin = "0.0", ClampMax = "360.0"))
    float FieldOfViewAngle = 120.0f;

    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "0.0"))
    float ObstacleAvoidanceWeight = 2.0f;

    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "1.0"))
    float ObstacleDetectionDistance = 300.0f;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "1", ClampMax = "12"))
    int32 NumberOfRaycasts = 8;
    
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
    bool bEnableObstacleAvoidance = true;
};

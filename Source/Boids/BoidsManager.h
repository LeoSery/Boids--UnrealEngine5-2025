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
    virtual void Tick(const float DeltaTime) override;
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    
    void SpawnBoids();
    FVector ConstrainPositionToBox(const FVector& Position) const;
    
    void SyncParametersToSystem() const;
    
    //// General
    UFUNCTION(BlueprintPure, Category = "Boids|General")
    FORCEINLINE int32 GetNumberOfBoids() const { return NumberOfBoids; }
    UFUNCTION(BlueprintPure, Category = "Boids|General")
    FORCEINLINE float GetBoidVelocity() const { return Velocity; }

    //// Perception
    UFUNCTION(BlueprintPure, Category = "Boids|Perception")
    FORCEINLINE float GetPerceptionRadius() const { return PerceptionRadius; }
    UFUNCTION(BlueprintPure, Category = "Boids|Perception")
    FORCEINLINE float GetFieldOfViewAngle() const { return FieldOfViewAngle; }

    //// Behavior
    // Separation
    UFUNCTION(BlueprintPure, Category = "Boids|Behavior|Separation")
    FORCEINLINE float GetSeparationWeight() const { return SeparationWeight; }
    UFUNCTION(BlueprintPure, Category = "Boids|Behavior|Separation")
    FORCEINLINE float GetSeparationRadius() const { return SeparationRadius; }
    
    // Alignment
    UFUNCTION(BlueprintPure, Category = "Boids|Behavior|Alignment")
    FORCEINLINE float GetAlignmentWeight() const { return AlignmentWeight; }
    
    // Cohesion
    UFUNCTION(BlueprintPure, Category = "Boids|Behavior|Cohesion")
    FORCEINLINE float GetCohesionWeight() const { return CohesionWeight; }
    
    //// Avoidance
    // Obstacle
    UFUNCTION(BlueprintPure, Category = "Boids|Obstacle Avoidance")
    FORCEINLINE bool GetEnableObstacleAvoidance() const { return bEnableObstacleAvoidance; }
    UFUNCTION(BlueprintPure, Category = "Boids|Obstacle Avoidance")
    FORCEINLINE float GetObstacleAvoidanceWeight() const { return ObstacleAvoidanceWeight; }
    UFUNCTION(BlueprintPure, Category = "Boids|Obstacle Avoidance")
    FORCEINLINE float GetObstacleDetectionDistance() const { return ObstacleDetectionDistance; }
    UFUNCTION(BlueprintPure, Category = "Boids|Obstacle Avoidance")
    FORCEINLINE int32 GetNumberOfRaycasts() const { return NumberOfRaycasts; }
    
    // Boundary
    UFUNCTION(BlueprintPure, Category = "Boids|Obstacle Avoidance")
    FORCEINLINE float GetBoundaryWeight() const { return BoundaryWeight; }
    
    //// General
    UFUNCTION(BlueprintCallable, Category = "Boids|General")
    FORCEINLINE void SetNumberOfBoids(const int32 NewValue);
    
    UFUNCTION(BlueprintCallable, Category = "Boids|General")
    void SetBoidVelocity(const float NewValue);

    //// Behavior
    // Perception
    UFUNCTION(BlueprintCallable, Category = "Boids|Perception")
    void SetPerceptionRadius(const float NewValue);

    UFUNCTION(BlueprintCallable, Category = "Boids|Perception")
    void SetFieldOfViewAngle(const float NewValue);

    // Separation
    UFUNCTION(BlueprintCallable, Category = "Boids|Behavior|Separation")
    void SetSeparationWeight(const float NewValue);
    UFUNCTION(BlueprintCallable, Category = "Boids|Behavior|Separation")
    void SetSeparationRadius(const float NewValue);

    // Alignment
    UFUNCTION(BlueprintCallable, Category = "Boids|Behavior|Alignment")
    void SetAlignmentWeight(const float NewValue);

    // Cohesion
    UFUNCTION(BlueprintCallable, Category = "Boids|Behavior|Cohesion")
    void SetCohesionWeight(const float NewValue);

    ////Avoidance
    UFUNCTION(BlueprintCallable, Category = "Boids|Obstacle Avoidance")
    void SetEnableObstacleAvoidance(const bool bNewValue);
    UFUNCTION(BlueprintCallable, Category = "Boids|Obstacle Avoidance")
    void SetObstacleAvoidanceWeight(const float NewValue);
    UFUNCTION(BlueprintCallable, Category = "Boids|Obstacle Avoidance")
    void SetObstacleDetectionDistance(const float NewValue);
    UFUNCTION(BlueprintCallable, Category = "Boids|Obstacle Avoidance")
    void SetNumberOfRaycasts(const int32 NewValue);
    // Boundary
    UFUNCTION(BlueprintCallable ,Category = "Boids|Obstacle Avoidance")
    void SetBoundaryWeight(const float NewValue);
    
    UFUNCTION(BlueprintCallable, Category = "Boids|Spawning")
    void RespawnBoids();
    
    UPROPERTY(EditAnywhere, Category = "Boids|Spawning")
    TSubclassOf<ABoid> BoidPrefab;
    
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Boids|Spawning")
    UBoxComponent* SpawnVolume;

protected:
    UPROPERTY()
    UBoidSystem* BoidSystem;

private:
    //// General
    UPROPERTY(EditAnywhere, Category = "Boids|General", meta = (ClampMin = "1", ClampMax = "1000"))
    int32 NumberOfBoids = 100;
    UPROPERTY(EditAnywhere, Category = "Boids|General", meta = (ClampMin = "1.0", ClampMax = "2000.0"))
    float Velocity = 1000.0f;

    //// Perception
    UPROPERTY(EditAnywhere, Category = "Boids|Perception", meta = (ClampMin = "1.0", ClampMax = "5000.0"))
    float PerceptionRadius = 200.0f;
    UPROPERTY(EditAnywhere, Category = "Boids|Perception", meta = (ClampMin = "1.0", ClampMax = "360.0"))
    float FieldOfViewAngle = 120.0f;

    //// Behavior
    // Separation
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior|Separation", meta = (ClampMin = "0.1", ClampMax = "20.0"))
    float SeparationWeight = 1.5f;
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior|Separation", meta = (ClampMin = "1.0", ClampMax = "250.0"))
    float SeparationRadius = 100.0f;

    // Alignment
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior|Alignment", meta = (ClampMin = "0.1", ClampMax = "20.0"))
    float AlignmentWeight = 1.0f;

    // Cohesion
    UPROPERTY(EditAnywhere, Category = "Boids|Behavior|Cohesion", meta = (ClampMin = "0.1", ClampMax = "20.0"))
    float CohesionWeight = 1.0f;
    
    //// Avoidance
    // Obstacle
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance")
    bool bEnableObstacleAvoidance = true;
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "0.1", ClampMax = "20.0", EditCondition = "bEnableObstacleAvoidance"))
    float ObstacleAvoidanceWeight = 2.0f;
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "0.1", ClampMax = "1000.0", EditCondition = "bEnableObstacleAvoidance"))
    float ObstacleDetectionDistance = 300.0f;
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "1", ClampMax = "12", EditCondition = "bEnableObstacleAvoidance"))
    int32 NumberOfRaycasts = 8;

    // Boundary
    UPROPERTY(EditAnywhere, Category = "Boids|Obstacle Avoidance", meta = (ClampMin = "0.0"))
    float BoundaryWeight = 1.0f;
};

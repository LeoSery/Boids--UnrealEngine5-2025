#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "BoidSystem.generated.h"

class ABoid;
class ABoidsManager;

UCLASS()
class BOIDS_API UBoidSystem : public UObject
{
	GENERATED_BODY()

public:
    UBoidSystem();
    
    void Initialize(const int32 NumBoids, const TArray<FVector>& InitialPositions, const TArray<FVector>& InitialDirections);
    void Update(const float DeltaTime);
    
    FORCEINLINE FVector GetPosition(const int32 Index) const { return Positions.IsValidIndex(Index) ? Positions[Index] : FVector::ZeroVector; }
    FORCEINLINE FVector GetDirection(const int32 Index) const { return Directions.IsValidIndex(Index) ? Directions[Index] : FVector::ZeroVector; }
    
    FORCEINLINE void SetPosition(const int32 Index, const FVector& Position) { if (Positions.IsValidIndex(Index)) Positions[Index] = Position; }
    FORCEINLINE void SetDirection(const int32 Index, const FVector& Direction) { if (Directions.IsValidIndex(Index)) Directions[Index] = Direction; }
    
    FORCEINLINE void SetActor(const int32 Index, ABoid* Actor) { if (BoidActors.IsValidIndex(Index)) BoidActors[Index] = Actor; }

    FORCEINLINE const TArray<ABoid*>& GetActors() const { return BoidActors; }
    FORCEINLINE TArray<ABoid*>& GetActors() { return BoidActors; }
    
    void SetBehaviorParameters(const float InSeparationWeight, const float InAlignmentWeight, const float InCohesionWeight, const float InSeparationRadius, const float InPerceptionRadius,
        const float InBoundaryWeight, const float InVelocity, const float InFieldOfViewAngle);

    void SetObstacleAvoidanceParameters(const float InObstacleAvoidanceWeight, const float InObstacleDetectionDistance, 
        const int32 InNumberOfRaycasts, const bool bInEnableObstacleAvoidance);

    UPROPERTY()
    ABoidsManager* OwnerManager;

    void GenerateRaycastRotators();

private:
    UPROPERTY()
    TArray<FVector> Positions;
    
    UPROPERTY()
    TArray<FVector> Directions;

    UPROPERTY()
    TArray<ABoid*> BoidActors;

    UPROPERTY()
    TArray<FRotator> RaycastRotators;

    TArray<FVector> CachedSeparationForces;
    TArray<FVector> CachedAlignmentForces;
    TArray<FVector> CachedCohesionForces;
    TArray<FVector> CachedBoundaryForces;
    TArray<FVector> CachedObstacleAvoidanceForces;

    FCollisionQueryParams GlobalObstacleQueryParams;
    
    float SeparationWeight;
    float AlignmentWeight;
    float CohesionWeight;
    float SeparationRadius;
    float PerceptionRadius;
    float BoundaryWeight;
    float Velocity;
    float FieldOfViewAngle;
    float FOVDotProductThreshold;
    float ObstacleAvoidanceWeight;
    bool bEnableObstacleAvoidance;
    float ObstacleDetectionDistance;
    int32 NumberOfRaycasts;
    
    // void CalculateSeparationForces(TArray<FVector>& OutForces) const;
    // void CalculateAlignmentForces(TArray<FVector>& OutForces) const;
    // void CalculateCohesionForces(TArray<FVector>& OutForces) const;
    void CalculateFlockingForces(TArray<FVector>& OutSeparationForces, TArray<FVector>& OutAlignmentForces, TArray<FVector>& OutCohesionForces) const;
    
    void CalculateBoundaryForces(TArray<FVector>& OutForces) const;
    void CalculateObstacleAvoidanceForces(TArray<FVector>& OutForces) const;
    
    void UpdatePositions(const float DeltaTime);
    
    FORCEINLINE bool AreNeighbors(const int32 BoidA, const int32 BoidB, const float Radius) const;

    struct FBoidNeighborCache
    {
        TArray<TArray<int32>> Neighbors;
        FORCEINLINE void Initialize(const int32 NumBoids) { Neighbors.SetNum(NumBoids); }
        FORCEINLINE void Clear() { for (auto& List : Neighbors) List.Empty(); }
    };
    
    FBoidNeighborCache NeighborCache;
    void FindAllNeighbors();
};

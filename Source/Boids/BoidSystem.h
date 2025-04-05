#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "BoidSystem.generated.h"

//////// FORWARD DECLARATION ////////
class ABoid;
class ABoidsManager;

//////// CLASS ////////
/// Core simulation system that handles boid behavior calculations and physics
UCLASS()
class BOIDS_API UBoidSystem : public UObject
{
	GENERATED_BODY()

public:
    //////// UNREAL LIFECYCLE ///////
    UBoidSystem();

    //////// PROPERTIES ////////
    /// Managers
    UPROPERTY()
    ABoidsManager* OwnerManager;
    
    //////// METHODS ////////
    /// System
    void Initialize(const int32 NumBoids, const TArray<FVector>& InitialPositions, const TArray<FVector>& InitialDirections);
    
    void Update(const float DeltaTime);
    
    /// Behavior parameters
    void SetBehaviorParameters(const float InSeparationWeight, const float InAlignmentWeight, const float InCohesionWeight, const float InSeparationRadius, const float InPerceptionRadius,
        const float InBoundaryWeight, const float InVelocity, const float InFieldOfViewAngle);

    void SetObstacleAvoidanceParameters(const float InObstacleAvoidanceWeight, const float InObstacleDetectionDistance, 
        const int32 InNumberOfRaycasts, const bool bInEnableObstacleAvoidance);

    /// Obstacle Avoidance
    void GenerateRaycastRotators();

    //////// GETTERS ////////
    /// Positioning
    FORCEINLINE FVector GetPosition(const int32 Index) const { return Positions.IsValidIndex(Index) ? Positions[Index] : FVector::ZeroVector; }
    
    FORCEINLINE FVector GetDirection(const int32 Index) const { return Directions.IsValidIndex(Index) ? Directions[Index] : FVector::ZeroVector; }

    /// Obstacle Avoidance
    FORCEINLINE float GetObstacleDetectionDistance() const { return ObstacleDetectionDistance; }
    
    FORCEINLINE bool IsObstacleAvoidanceEnabled() const { return bEnableObstacleAvoidance; }

    /// Actor
    FORCEINLINE const TArray<ABoid*>& GetActors() const { return BoidActors; }
    
    FORCEINLINE TArray<ABoid*>& GetActors() { return BoidActors; }

    /// Debug
    FORCEINLINE const TArray<FRotator>& GetRaycastRotators() const { return RaycastRotators; }
    
    FORCEINLINE bool GetDebugDraw() const { return bEnableObstacleAvoidance; }

    //////// SETTERS ////////
    /// Positioning
    FORCEINLINE void SetPosition(const int32 Index, const FVector& Position) { if (Positions.IsValidIndex(Index)) Positions[Index] = Position; }
    
    FORCEINLINE void SetDirection(const int32 Index, const FVector& Direction) { if (Directions.IsValidIndex(Index)) Directions[Index] = Direction; }

    /// Boids
    FORCEINLINE void SetActor(const int32 Index, ABoid* Actor) { if (BoidActors.IsValidIndex(Index)) BoidActors[Index] = Actor; }

    /// Obstacle Avoidance
    void SetUseUniformDistribution(const bool bInUseUniformDistribution);

private:
    //////// STRUCTS ////////
    /// Cache
    struct FBoidNeighborCache
    {
        TArray<TArray<int32>> Neighbors;
        FORCEINLINE void Initialize(const int32 NumBoids) { Neighbors.SetNum(NumBoids); }
        FORCEINLINE void Clear() { for (auto& List : Neighbors) List.Empty(); }
    };
    
    // Profiling
    struct FProfilingData
    {
        double UpdateTotal = 0.0;
        double FindNeighborsTime = 0.0;
        double FlockingForcesTime = 0.0;
        double BoundaryForcesTime = 0.0;
        double ObstacleAvoidanceTime = 0.0;
        double UpdatePositionsTime = 0.0;
        
        void LogResults() const
        {
            UE_LOG(LogTemp, Warning, TEXT("--- BOIDS PROFILING ---"));
            UE_LOG(LogTemp, Warning, TEXT("Update total: %.3f ms"), UpdateTotal * 1000.0);
            UE_LOG(LogTemp, Warning, TEXT("FindNeighbors: %.3f ms (%.1f%%)"), 
                FindNeighborsTime * 1000.0, (FindNeighborsTime / UpdateTotal) * 100.0);
            UE_LOG(LogTemp, Warning, TEXT("FlockingForces: %.3f ms (%.1f%%)"), 
                FlockingForcesTime * 1000.0, (FlockingForcesTime / UpdateTotal) * 100.0);
            UE_LOG(LogTemp, Warning, TEXT("BoundaryForces: %.3f ms (%.1f%%)"), 
                BoundaryForcesTime * 1000.0, (BoundaryForcesTime / UpdateTotal) * 100.0);
            UE_LOG(LogTemp, Warning, TEXT("ObstacleAvoidance: %.3f ms (%.1f%%)"), 
                ObstacleAvoidanceTime * 1000.0, (ObstacleAvoidanceTime / UpdateTotal) * 100.0);
            UE_LOG(LogTemp, Warning, TEXT("UpdatePositions: %.3f ms (%.1f%%)"), 
                UpdatePositionsTime * 1000.0, (UpdatePositionsTime / UpdateTotal) * 100.0);
        }
    };
    
    //////// METHODS ////////
    /// Forces
    void CalculateFlockingForces(TArray<FVector>& OutSeparationForces, TArray<FVector>& OutAlignmentForces, TArray<FVector>& OutCohesionForces) const;
    
    void CalculateBoundaryForces(TArray<FVector>& OutForces) const;
    
    void CalculateObstacleAvoidanceForces(TArray<FVector>& OutForces) const;

    /// Rendering
    void UpdatePositions(const float DeltaTime);

    /// Compute
    bool AreNeighbors(const int32 BoidA, const int32 BoidB, const float Radius) const;
    
    void FindAllNeighbors();
    
    //////// PROPERTIES ////////
    /// Data oriented
    // Boids
    UPROPERTY()
    TArray<FVector> Positions;
    
    UPROPERTY()
    TArray<FVector> Directions;

    UPROPERTY()
    TArray<ABoid*> BoidActors;

    UPROPERTY()
    TArray<FRotator> RaycastRotators;

    // Forces
    TArray<FVector> CachedSeparationForces;
    TArray<FVector> CachedAlignmentForces;
    TArray<FVector> CachedCohesionForces;
    TArray<FVector> CachedBoundaryForces;
    TArray<FVector> CachedObstacleAvoidanceForces;

    /// Collision
    bool bUseUniformDistribution;
    FCollisionQueryParams GlobalObstacleQueryParams;

    /// Cache
    FBoidNeighborCache NeighborCache;
    int32 LastFrameMaxNeighbors = 32;

    /// Profiling
    FProfilingData ProfilingData;
    bool bEnableProfiling = true;

    /// System
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
};

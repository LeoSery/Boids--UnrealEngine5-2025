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
    
    FORCEINLINE const TArray<FVector>& GetPositions() const { return Positions; }
    FORCEINLINE const TArray<FVector>& GetDirections() const { return Directions; }
    FORCEINLINE int32 GetCount() const { return Positions.Num(); }

    FORCEINLINE float GetObstacleDetectionDistance() const { return ObstacleDetectionDistance; }
    FORCEINLINE bool IsObstacleAvoidanceEnabled() const { return bEnableObstacleAvoidance; }

    FORCEINLINE const TArray<FRotator>& GetRaycastRotators() const { return RaycastRotators; }
    FORCEINLINE bool GetDebugDraw() const { return bEnableObstacleAvoidance; }
    
    void SetBehaviorParameters(const float InSeparationWeight, const float InAlignmentWeight, const float InCohesionWeight, const float InSeparationRadius, const float InPerceptionRadius,
        const float InBoundaryWeight, const float InVelocity, const float InFieldOfViewAngle);

    void SetObstacleAvoidanceParameters(const float InObstacleAvoidanceWeight, const float InObstacleDetectionDistance, 
        const int32 InNumberOfRaycasts, const bool bInEnableObstacleAvoidance);

    UPROPERTY()
    ABoidsManager* OwnerManager;

    void GenerateRaycastRotators();
    void SetUseUniformDistribution(const bool bInUseUniformDistribution);

private:
    UPROPERTY()
    TArray<FVector> Positions;
    
    UPROPERTY()
    TArray<FVector> Directions;

    UPROPERTY()
    TArray<FRotator> RaycastRotators;

    TArray<FVector> CachedSeparationForces;
    TArray<FVector> CachedAlignmentForces;
    TArray<FVector> CachedCohesionForces;
    TArray<FVector> CachedBoundaryForces;
    TArray<FVector> CachedObstacleAvoidanceForces;

    bool bUseUniformDistribution;
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
    
    void CalculateFlockingForces(TArray<FVector>& OutSeparationForces, TArray<FVector>& OutAlignmentForces, TArray<FVector>& OutCohesionForces) const;
    
    void CalculateBoundaryForces(TArray<FVector>& OutForces) const;
    void CalculateObstacleAvoidanceForces(TArray<FVector>& OutForces) const;
    
    void UpdatePositions(const float DeltaTime);
    
    bool AreNeighbors(const int32 BoidA, const int32 BoidB, const float Radius) const;

    struct FBoidNeighborCache
    {
        TArray<TArray<int32>> Neighbors;
        FORCEINLINE void Initialize(const int32 NumBoids) { Neighbors.SetNum(NumBoids); }
        FORCEINLINE void Clear() { for (auto& List : Neighbors) List.Empty(); }
    };
    
    FBoidNeighborCache NeighborCache;
    int32 LastFrameMaxNeighbors = 32;
    void FindAllNeighbors();
    
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
            UE_LOG(LogTemp, Warning, TEXT("--- PROFIL BOIDS ---"));
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
    
    FProfilingData ProfilingData;
    bool bEnableProfiling = true;
};

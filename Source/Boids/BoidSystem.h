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
    
    void Initialize(int32 NumBoids, const TArray<FVector>& InitialPositions, const TArray<FVector>& InitialDirections);
    void Update(float DeltaTime);
    
    FORCEINLINE FVector GetPosition(int32 Index) const { return Positions.IsValidIndex(Index) ? Positions[Index] : FVector::ZeroVector; }
    FORCEINLINE FVector GetDirection(int32 Index) const { return Directions.IsValidIndex(Index) ? Directions[Index] : FVector::ZeroVector; }
    
    FORCEINLINE void SetPosition(int32 Index, const FVector& Position) { if (Positions.IsValidIndex(Index)) Positions[Index] = Position; }
    FORCEINLINE void SetDirection(int32 Index, const FVector& Direction) { if (Directions.IsValidIndex(Index)) Directions[Index] = Direction; }
    
    FORCEINLINE void SetActor(int32 Index, ABoid* Actor) { if (BoidActors.IsValidIndex(Index)) BoidActors[Index] = Actor; }
    FORCEINLINE TArray<ABoid*>& GetActors() { return BoidActors; }
    
    void SetBehaviorParameters(float InSeparationWeight, float InAlignmentWeight, float InCohesionWeight, float InSeparationRadius, float InPerceptionRadius,
        float InBoundaryWeight, float InVelocity, float InFieldOfViewAngle);

    void SetObstacleAvoidanceParameters(float InObstacleAvoidanceWeight, float InObstacleDetectionDistance, 
        int32 InNumberOfRaycasts, bool bInEnableObstacleAvoidance);

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
    
    void CalculateSeparationForces(TArray<FVector>& OutForces);
    void CalculateAlignmentForces(TArray<FVector>& OutForces);
    void CalculateCohesionForces(TArray<FVector>& OutForces);
    void CalculateBoundaryForces(TArray<FVector>& OutForces);
    void CalculateObstacleAvoidanceForces(TArray<FVector>& OutForces);
    
    void UpdatePositions(float DeltaTime);
    
    FORCEINLINE bool AreNeighbors(int32 BoidA, int32 BoidB, float Radius) const;
    
    struct FBoidNeighborCache
    {
        TArray<TArray<int32>> Neighbors;
        FORCEINLINE void Initialize(int32 NumBoids) { Neighbors.SetNum(NumBoids); }
        FORCEINLINE void Clear() { for (auto& List : Neighbors) List.Empty(); }
    };
    
    FBoidNeighborCache NeighborCache;
    void FindAllNeighbors();
};

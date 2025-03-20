#include "BoidSystem.h"
#include "BoidsManager.h"
#include "Boid.h"

UBoidSystem::UBoidSystem() : OwnerManager(nullptr)
{
    
}

void UBoidSystem::Initialize(int32 NumBoids, const TArray<FVector>& InitialPositions, const TArray<FVector>& InitialDirections)
{
    Positions.SetNum(NumBoids);
    Directions.SetNum(NumBoids);
    BoidActors.SetNum(NumBoids);
    
    for (int32 i = 0; i < NumBoids; ++i)
    {
        Positions[i] = InitialPositions.IsValidIndex(i) ? InitialPositions[i] : FVector::ZeroVector;
        Directions[i] = InitialDirections.IsValidIndex(i) ? InitialDirections[i].GetSafeNormal() : FVector(1.0f, 0.0f, 0.0f);
    }
    
    NeighborCache.Initialize(NumBoids);
}

void UBoidSystem::SetBehaviorParameters(float InSeparationWeight, float InAlignmentWeight, float InCohesionWeight, float InSeparationRadius, float InPerceptionRadius,
    float InBoundaryWeight, float InVelocity, float InFieldOfViewAngle)
{
    SeparationWeight = InSeparationWeight;
    AlignmentWeight = InAlignmentWeight;
    CohesionWeight = InCohesionWeight;
    SeparationRadius = InSeparationRadius;
    PerceptionRadius = InPerceptionRadius;
    BoundaryWeight = InBoundaryWeight;
    Velocity = InVelocity;
    FieldOfViewAngle = InFieldOfViewAngle;

    FOVDotProductThreshold = FMath::Cos(FMath::DegreesToRadians(FieldOfViewAngle * 0.5f));
}

void UBoidSystem::Update(float DeltaTime)
{
    FindAllNeighbors();
    
    TArray<FVector> SeparationForces;
    TArray<FVector> AlignmentForces;
    TArray<FVector> CohesionForces;
    TArray<FVector> BoundaryForces;
    TArray<FVector> ObstacleAvoidanceForces;
    
    SeparationForces.SetNum(Positions.Num());
    AlignmentForces.SetNum(Positions.Num());
    CohesionForces.SetNum(Positions.Num());
    BoundaryForces.SetNum(Positions.Num());
    ObstacleAvoidanceForces.SetNum(Positions.Num());
    
    CalculateSeparationForces(SeparationForces);
    CalculateAlignmentForces(AlignmentForces);
    CalculateCohesionForces(CohesionForces);
    CalculateBoundaryForces(BoundaryForces);

    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        if (BoidActors[i])
        {
            ObstacleAvoidanceForces[i] = BoidActors[i]->ComputeObstacleAvoidance();
        }
    }
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        FVector TotalForce = FVector::ZeroVector;
        
        if (!SeparationForces[i].IsNearlyZero())
            TotalForce += SeparationForces[i] * SeparationWeight;
        
        if (!AlignmentForces[i].IsNearlyZero())
            TotalForce += AlignmentForces[i] * AlignmentWeight;
        
        if (!CohesionForces[i].IsNearlyZero())
            TotalForce += CohesionForces[i] * CohesionWeight;

        if (!BoundaryForces[i].IsNearlyZero())
            TotalForce += BoundaryForces[i] * BoundaryWeight;

        if (!ObstacleAvoidanceForces[i].IsNearlyZero() && BoidActors[i])
            TotalForce += ObstacleAvoidanceForces[i] * BoidActors[i]->ObstacleAvoidanceWeight;
        
        if (TotalForce.IsNearlyZero())
        {
            continue;
        }
        
        TotalForce.Normalize();
        
        Directions[i] = FMath::VInterpNormalRotationTo(
            Directions[i],
            TotalForce,
            DeltaTime,
            90.0f
        );
    }
    
    UpdatePositions(DeltaTime);
}

void UBoidSystem::FindAllNeighbors()
{
    NeighborCache.Clear();
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        for (int32 j = 0; j < Positions.Num(); ++j)
        {
            if (i == j)
            {
                continue;
            }
            
            float DistSquared = FVector::DistSquared(Positions[i], Positions[j]);
            
            if (DistSquared <= PerceptionRadius * PerceptionRadius)
            {
                if (BoidActors[i])
                {
                    FVector DirectionToOther = (Positions[j] - Positions[i]).GetSafeNormal();
                    float DotProduct = FVector::DotProduct(Directions[i], DirectionToOther);
                    
                    if (DotProduct >= FOVDotProductThreshold)
                    {
                        NeighborCache.Neighbors[i].Add(j);
                    }
                }
                else
                {
                    NeighborCache.Neighbors[i].Add(j);
                }
            }
        }
    }
}

void UBoidSystem::CalculateSeparationForces(TArray<FVector>& OutForces)
{
    for (int32 i = 0; i < OutForces.Num(); ++i)
    {
        OutForces[i] = FVector::ZeroVector;
    }
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        int32 NeighborCount = 0;
        FVector Force = FVector::ZeroVector;
        
        for (int32 NeighborIdx : NeighborCache.Neighbors[i])
        {
            float Distance = FVector::Dist(Positions[i], Positions[NeighborIdx]);
            
            if (Distance < SeparationRadius && Distance > 0)
            {
                FVector AwayFromNeighbor = Positions[i] - Positions[NeighborIdx];
                AwayFromNeighbor.Normalize();
                
                AwayFromNeighbor *= (SeparationRadius / FMath::Max(Distance, 1.0f));
                
                Force += AwayFromNeighbor;
                NeighborCount++;
            }
        }
        
        if (NeighborCount > 0)
        {
            Force /= (float)NeighborCount;
            
            if (!Force.IsNearlyZero())
            {
                Force.Normalize();
            }
        }
        
        OutForces[i] = Force;
    }
}

void UBoidSystem::CalculateAlignmentForces(TArray<FVector>& OutForces)
{
    for (int32 i = 0; i < OutForces.Num(); ++i)
    {
        OutForces[i] = FVector::ZeroVector;
    }
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        if (NeighborCache.Neighbors[i].Num() == 0)
        {
            continue;
        }
        
        FVector AverageDirection = FVector::ZeroVector;
        
        for (int32 NeighborIdx : NeighborCache.Neighbors[i])
        {
            AverageDirection += Directions[NeighborIdx];
        }
        
        if (!AverageDirection.IsNearlyZero())
        {
            AverageDirection.Normalize();
        }
        
        OutForces[i] = AverageDirection;
    }
}

void UBoidSystem::CalculateCohesionForces(TArray<FVector>& OutForces)
{
    for (int32 i = 0; i < OutForces.Num(); ++i)
    {
        OutForces[i] = FVector::ZeroVector;
    }
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        if (NeighborCache.Neighbors[i].Num() == 0)
        {
            continue;
        }
        
        FVector CenterOfMass = FVector::ZeroVector;
        
        for (int32 NeighborIdx : NeighborCache.Neighbors[i])
        {
            CenterOfMass += Positions[NeighborIdx];
        }
        
        CenterOfMass /= NeighborCache.Neighbors[i].Num();
        
        FVector DirectionToCenter = CenterOfMass - Positions[i];
        
        if (!DirectionToCenter.IsNearlyZero())
        {
            DirectionToCenter.Normalize();
        }
        
        OutForces[i] = DirectionToCenter;
    }
}

void UBoidSystem::CalculateBoundaryForces(TArray<FVector>& OutForces)
{
    if (!OwnerManager)
    {
        return;
    }

    FVector BoxOrigin = OwnerManager->SpawnVolume->GetComponentLocation();
    FVector BoxExtent = OwnerManager->SpawnVolume->GetScaledBoxExtent();
    
    const float BoundaryMargin = 50.0f;
    const float ForceStrength = 1.0f;
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        FVector LocalPos = Positions[i] - BoxOrigin;
        FVector BoundaryForce = FVector::ZeroVector;
        
        // Calcul X
        if (LocalPos.X > BoxExtent.X - BoundaryMargin)
            BoundaryForce.X = -ForceStrength * (1.0f - ((BoxExtent.X - LocalPos.X) / BoundaryMargin));
        else if (LocalPos.X < -BoxExtent.X + BoundaryMargin)
            BoundaryForce.X = ForceStrength * (1.0f - ((-BoxExtent.X - LocalPos.X) / -BoundaryMargin));
            
        // Calcul Y
        if (LocalPos.Y > BoxExtent.Y - BoundaryMargin)
            BoundaryForce.Y = -ForceStrength * (1.0f - ((BoxExtent.Y - LocalPos.Y) / BoundaryMargin));
        else if (LocalPos.Y < -BoxExtent.Y + BoundaryMargin)
            BoundaryForce.Y = ForceStrength * (1.0f - ((-BoxExtent.Y - LocalPos.Y) / -BoundaryMargin));
            
        // Calcul Z
        if (LocalPos.Z > BoxExtent.Z - BoundaryMargin)
            BoundaryForce.Z = -ForceStrength * (1.0f - ((BoxExtent.Z - LocalPos.Z) / BoundaryMargin));
        else if (LocalPos.Z < -BoxExtent.Z + BoundaryMargin)
            BoundaryForce.Z = ForceStrength * (1.0f - ((-BoxExtent.Z - LocalPos.Z) / -BoundaryMargin));
        
        OutForces[i] = BoundaryForce;
    }
}

void UBoidSystem::UpdatePositions(float DeltaTime)
{
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        FVector Movement = Directions[i] * Velocity * DeltaTime;
        Positions[i] += Movement;
        
        if (BoidActors[i] && BoidActors[i]->BoidsManager)
        {
            Positions[i] = BoidActors[i]->BoidsManager->ConstrainPositionToBox(Positions[i]);
        }
        
        if (BoidActors[i])
        {
            BoidActors[i]->SetActorLocation(Positions[i]);
            BoidActors[i]->SetActorRotation(Directions[i].Rotation());
            BoidActors[i]->Direction = Directions[i];
        }
    }
}

FORCEINLINE bool UBoidSystem::AreNeighbors(int32 BoidA, int32 BoidB, float Radius) const
{
    if (BoidA == BoidB)
    {
        return false;
    }
    
    float DistSquared = FVector::DistSquared(Positions[BoidA], Positions[BoidB]);
    return DistSquared <= Radius * Radius;
}
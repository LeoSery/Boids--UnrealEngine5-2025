#include "BoidSystem.h"
#include "BoidsManager.h"
#include "Boid.h"

UBoidSystem::UBoidSystem() : OwnerManager(nullptr)
{
    NumberOfRaycasts = 8;
    GenerateRaycastRotators();
}

void UBoidSystem::Initialize(const int32 NumBoids, const TArray<FVector>& InitialPositions, const TArray<FVector>& InitialDirections)
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
    
    CachedSeparationForces.SetNum(NumBoids);
    CachedAlignmentForces.SetNum(NumBoids);
    CachedCohesionForces.SetNum(NumBoids);
    CachedBoundaryForces.SetNum(NumBoids);
    CachedObstacleAvoidanceForces.SetNum(NumBoids);
}

void UBoidSystem::SetBehaviorParameters(const float InSeparationWeight, const float InAlignmentWeight, const float InCohesionWeight, const float InSeparationRadius, const float InPerceptionRadius,
    const float InBoundaryWeight, const float InVelocity, const float InFieldOfViewAngle)
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

void UBoidSystem::SetObstacleAvoidanceParameters(const float InObstacleAvoidanceWeight, const float InObstacleDetectionDistance,
    const int32 InNumberOfRaycasts, const bool bInEnableObstacleAvoidance)
{
    ObstacleAvoidanceWeight = InObstacleAvoidanceWeight;
    ObstacleDetectionDistance = InObstacleDetectionDistance;

    if (NumberOfRaycasts != InNumberOfRaycasts)
    {
        NumberOfRaycasts = InNumberOfRaycasts;
        GenerateRaycastRotators();
    }
    
    bEnableObstacleAvoidance = bInEnableObstacleAvoidance;
}

void UBoidSystem::Update(const float DeltaTime)
{
    FindAllNeighbors();
    
    const int32 NumBoids = Positions.Num();
    if (CachedSeparationForces.Num() < NumBoids)
    {
        CachedSeparationForces.SetNum(NumBoids);
        CachedAlignmentForces.SetNum(NumBoids);
        CachedCohesionForces.SetNum(NumBoids);
        CachedBoundaryForces.SetNum(NumBoids);
        CachedObstacleAvoidanceForces.SetNum(NumBoids);
    }

    // Fast memory zeroing
    FMemory::Memzero(CachedSeparationForces.GetData(), NumBoids * sizeof(FVector));
    FMemory::Memzero(CachedAlignmentForces.GetData(), NumBoids * sizeof(FVector));
    FMemory::Memzero(CachedCohesionForces.GetData(), NumBoids * sizeof(FVector));
    FMemory::Memzero(CachedBoundaryForces.GetData(), NumBoids * sizeof(FVector));
    FMemory::Memzero(CachedObstacleAvoidanceForces.GetData(), NumBoids * sizeof(FVector));
    
    CalculateSeparationForces(CachedSeparationForces);
    CalculateAlignmentForces(CachedAlignmentForces);
    CalculateCohesionForces(CachedCohesionForces);
    CalculateBoundaryForces(CachedBoundaryForces);
    CalculateObstacleAvoidanceForces(CachedObstacleAvoidanceForces);
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        FVector TotalForce = FVector::ZeroVector;
        
        if (!CachedSeparationForces[i].IsNearlyZero())
            TotalForce += CachedSeparationForces[i] * SeparationWeight;
        
        if (!CachedAlignmentForces[i].IsNearlyZero())
            TotalForce += CachedAlignmentForces[i] * AlignmentWeight;
        
        if (!CachedCohesionForces[i].IsNearlyZero())
            TotalForce += CachedCohesionForces[i] * CohesionWeight;

        if (!CachedBoundaryForces[i].IsNearlyZero())
            TotalForce += CachedBoundaryForces[i] * BoundaryWeight;

        if (!CachedObstacleAvoidanceForces[i].IsNearlyZero() && BoidActors[i])
            TotalForce += CachedObstacleAvoidanceForces[i] * ObstacleAvoidanceWeight;
        
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
    
    const float PerceptionRadiusSq = PerceptionRadius * PerceptionRadius;
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        for (int32 j = i + 1; j < Positions.Num(); ++j)
        {
            const float DistSquared = FVector::DistSquared(Positions[i], Positions[j]);
            
            if (DistSquared <= PerceptionRadiusSq)
            {
                // Tester si j est dans le FOV de i
                const FVector DirectionItoJ = (Positions[j] - Positions[i]).GetSafeNormal();
                const float DotProductI = FVector::DotProduct(Directions[i], DirectionItoJ);
                
                if (DotProductI >= FOVDotProductThreshold)
                {
                    NeighborCache.Neighbors[i].Add(j);
                }
                
                // Tester si i est dans le FOV de j (utilise la symétrie de distance)
                const FVector DirectionJtoI = -DirectionItoJ;
                const float DotProductJ = FVector::DotProduct(Directions[j], DirectionJtoI);
                
                if (DotProductJ >= FOVDotProductThreshold)
                {
                    NeighborCache.Neighbors[j].Add(i);
                }
            }
        }
    }
}

void UBoidSystem::CalculateSeparationForces(TArray<FVector>& OutForces) const
{
    for (int32 i = 0; i < OutForces.Num(); ++i)
    {
        OutForces[i] = FVector::ZeroVector;
    }
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        int32 NeighborCount = 0;
        FVector Force = FVector::ZeroVector;
        
        for (const int32 NeighborIdx : NeighborCache.Neighbors[i])
        {
            const float Distance = FVector::Dist(Positions[i], Positions[NeighborIdx]);
            
            if (Distance < SeparationRadius && Distance > 0)
            {
                FVector AwayFromNeighbor = Positions[i] - Positions[NeighborIdx];
                AwayFromNeighbor.Normalize();
                
                AwayFromNeighbor *= SeparationRadius / FMath::Max(Distance, 1.0f);
                
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

void UBoidSystem::CalculateAlignmentForces(TArray<FVector>& OutForces) const
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
        
        for (const int32 NeighborIdx : NeighborCache.Neighbors[i])
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

void UBoidSystem::CalculateCohesionForces(TArray<FVector>& OutForces) const
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
        
        for (const int32 NeighborIdx : NeighborCache.Neighbors[i])
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

void UBoidSystem::CalculateBoundaryForces(TArray<FVector>& OutForces) const
{
    if (!OwnerManager)
    {
        return;
    }

    const FVector BoxOrigin = OwnerManager->SpawnVolume->GetComponentLocation();
    const FVector BoxExtent = OwnerManager->SpawnVolume->GetScaledBoxExtent();
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        constexpr float BoundaryMargin = 50.0f;
        constexpr float ForceStrength = 1.0f;
        
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

void UBoidSystem::CalculateObstacleAvoidanceForces(TArray<FVector>& OutForces) const
{
    if (!bEnableObstacleAvoidance || !OwnerManager)
    {
        return;
    }

    const UWorld* World = OwnerManager->GetWorld();
    if (!World)
    {
        return;
    }
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        OutForces[i] = FVector::ZeroVector;
        
        FVector AvoidanceForce = FVector::ZeroVector;
        FVector BoidPosition = Positions[i];
        FVector BoidDirection = Directions[i];
        int32 HitCount = 0;
        
        FCollisionQueryParams QueryParams;
        
        // Ignorer tous les acteurs de boids
        for (const ABoid* Boid : BoidActors)
        {
            if (Boid)
            {
                QueryParams.AddIgnoredActor(Boid);
            }
        }
        
        for (const FRotator& Rotator : RaycastRotators)
        {
            FVector WorldDir = Rotator.RotateVector(BoidDirection);
            
            FHitResult HitResult;
            
            const bool bHit = World->LineTraceSingleByChannel(
                HitResult,
                BoidPosition,
                BoidPosition + WorldDir * ObstacleDetectionDistance,
                ECC_WorldStatic,
                QueryParams
            );
            
            // Visualisation pour debug (optionnel)
            /*
            DrawDebugLine(World, BoidPosition, BoidPosition + WorldDir * ObstacleDetectionDistance, 
                          bHit ? FColor::Red : FColor::Green, false, -1.0f, 0, 1.0f);
            */
            
            if (bHit)
            {
                const float Distance = HitResult.Distance;
                const float StrengthFactor = 1.0f - (Distance / ObstacleDetectionDistance);
                
                const FVector AwayFromObstacle = -WorldDir * StrengthFactor * 2.0f;
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
        
        OutForces[i] = AvoidanceForce;
    }
}

void UBoidSystem::UpdatePositions(const float DeltaTime)
{
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        const FVector Movement = Directions[i] * Velocity * DeltaTime;
        Positions[i] += Movement;
        
        if (BoidActors[i] && BoidActors[i]->BoidsManager)
        {
            Positions[i] = BoidActors[i]->BoidsManager->ConstrainPositionToBox(Positions[i]);
        }
        
        if (BoidActors[i])
        {
            BoidActors[i]->SetActorLocation(Positions[i]);
            BoidActors[i]->SetActorRotation(Directions[i].Rotation());
        }
    }
}

void UBoidSystem::GenerateRaycastRotators()
{
    RaycastRotators.Empty();
    
    RaycastRotators.Add(FRotator::ZeroRotator);

    constexpr float YawAngle = 30.0f;    // Angle horizontal
    constexpr float PitchAngle = 30.0f;  // Angle vertical
    
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
        constexpr float HalfYaw = YawAngle * 0.5f;
        constexpr float HalfPitch = PitchAngle * 0.5f;
        
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

FORCEINLINE bool UBoidSystem::AreNeighbors(const int32 BoidA, const int32 BoidB, const float Radius) const
{
    if (BoidA == BoidB)
    {
        return false;
    }
    
    const float DistSquared = FVector::DistSquared(Positions[BoidA], Positions[BoidB]);
    return DistSquared <= Radius * Radius;
}
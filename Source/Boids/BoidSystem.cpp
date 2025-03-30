#include "BoidSystem.h"
#include "BoidsManager.h"
#include "Boid.h"

UBoidSystem::UBoidSystem() : OwnerManager(nullptr)
{
    NumberOfRaycasts = 8;
    bUseUniformDistribution = false;
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
    
    LastUpdatedPositions.SetNum(NumBoids);

    GlobalObstacleQueryParams = FCollisionQueryParams::DefaultQueryParam;
    GlobalObstacleQueryParams.bTraceComplex = false;
    
    for (const ABoid* Boid : BoidActors)
    {
        if (Boid)
        {
            GlobalObstacleQueryParams.AddIgnoredActor(Boid);
        }
    }
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
    double StartUpdateTime = 0.0;
    if (bEnableProfiling)
    {
        StartUpdateTime = FPlatformTime::Seconds();
    }
    
    double StartNeighborsTime = 0.0;
    if (bEnableProfiling) StartNeighborsTime = FPlatformTime::Seconds();
    FindAllNeighbors();
    
    if (bEnableProfiling) ProfilingData.FindNeighborsTime = FPlatformTime::Seconds() - StartNeighborsTime;
    
    const int32 NumBoids = Positions.Num();
    if (CachedSeparationForces.Num() < NumBoids)
    {
        CachedSeparationForces.SetNum(NumBoids);
        CachedAlignmentForces.SetNum(NumBoids);
        CachedCohesionForces.SetNum(NumBoids);
        CachedBoundaryForces.SetNum(NumBoids);
        CachedObstacleAvoidanceForces.SetNum(NumBoids);
    }

    double StartFlockingTime = 0.0;
    if (bEnableProfiling) StartFlockingTime = FPlatformTime::Seconds();
    CalculateFlockingForces(CachedSeparationForces, CachedAlignmentForces, CachedCohesionForces);
    if (bEnableProfiling) ProfilingData.FlockingForcesTime = FPlatformTime::Seconds() - StartFlockingTime;

    double StartBoundaryTime = 0.0;
    if (bEnableProfiling) StartBoundaryTime = FPlatformTime::Seconds();
    CalculateBoundaryForces(CachedBoundaryForces);
    if (bEnableProfiling) ProfilingData.BoundaryForcesTime = FPlatformTime::Seconds() - StartBoundaryTime;

    double StartObstacleTime = 0.0;
    if (bEnableProfiling) StartObstacleTime = FPlatformTime::Seconds();
    CalculateObstacleAvoidanceForces(CachedObstacleAvoidanceForces);
    if (bEnableProfiling) ProfilingData.ObstacleAvoidanceTime = FPlatformTime::Seconds() - StartObstacleTime;
    
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

    double StartPositionsTime = 0.0;
    if (bEnableProfiling) StartPositionsTime = FPlatformTime::Seconds();
    
    UpdatePositions(DeltaTime);

    if (bEnableProfiling)
    {
        ProfilingData.UpdatePositionsTime = FPlatformTime::Seconds() - StartPositionsTime;
        
        ProfilingData.UpdateTotal = FPlatformTime::Seconds() - StartUpdateTime;
        
        static int32 FrameCounter = 0;
        if (++FrameCounter >= 60)
        {
            ProfilingData.LogResults();
            FrameCounter = 0;
        }
    }
}

void UBoidSystem::FindAllNeighbors()
{
    NeighborCache.Clear();
    
    const float PerceptionRadiusSq = PerceptionRadius * PerceptionRadius;
    const int32 NumBoids = Positions.Num();
    
    for (int32 i = 0; i < NumBoids; ++i)
    {
        for (int32 j = i + 1; j < NumBoids; ++j)
        {
            const float DistSquared = FVector::DistSquared(Positions[i], Positions[j]);
            
            if (DistSquared <= PerceptionRadiusSq)
            {
                const FVector DirectionItoJ = (Positions[j] - Positions[i]).GetSafeNormal();
                const float DotProductI = FVector::DotProduct(Directions[i], DirectionItoJ);
                
                const FVector DirectionJtoI = -DirectionItoJ;
                const float DotProductJ = FVector::DotProduct(Directions[j], DirectionJtoI);
                
                if (DotProductI >= FOVDotProductThreshold)
                {
                    NeighborCache.Neighbors[i].Add(j);
                }
                
                if (DotProductJ >= FOVDotProductThreshold)
                {
                    NeighborCache.Neighbors[j].Add(i);
                }
            }
        }
    }
    
    int32 MaxNeighborsThisFrame = 0;
    for (int32 i = 0; i < NumBoids; ++i)
    {
        MaxNeighborsThisFrame = FMath::Max(MaxNeighborsThisFrame, NeighborCache.Neighbors[i].Num());
    }
    
    LastFrameMaxNeighbors = FMath::Max(1, MaxNeighborsThisFrame + (MaxNeighborsThisFrame / 4));
}

void UBoidSystem::CalculateFlockingForces(TArray<FVector>& OutSeparationForces, TArray<FVector>& OutAlignmentForces, TArray<FVector>& OutCohesionForces) const
{
    // Fast memory zeroing
    FMemory::Memzero(OutSeparationForces.GetData(), OutSeparationForces.Num() * sizeof(FVector));
    FMemory::Memzero(OutAlignmentForces.GetData(), OutAlignmentForces.Num() * sizeof(FVector));
    FMemory::Memzero(OutCohesionForces.GetData(), OutCohesionForces.Num() * sizeof(FVector));

    if (Positions.Num() <= 1 || !OwnerManager)
    {
        return;
    }
    
    const int32 NumBoids = Positions.Num();
    
    ParallelFor(NumBoids, [&](int32 i)
    {
        if (NeighborCache.Neighbors[i].Num() == 0)
        {
            return;
        }
        
        int32 SeparationCount = 0;
        FVector SeparationForce = FVector::ZeroVector;
        FVector AverageDirection = FVector::ZeroVector;
        FVector CenterOfMass = FVector::ZeroVector;
        
        for (const int32 NeighborIdx : NeighborCache.Neighbors[i])
        {
            // Separation
            const float Distance = FVector::Dist(Positions[i], Positions[NeighborIdx]);
            if (Distance < SeparationRadius && Distance > 0)
            {
                FVector AwayFromNeighbor = Positions[i] - Positions[NeighborIdx];
                AwayFromNeighbor.Normalize();
                AwayFromNeighbor *= SeparationRadius / FMath::Max(Distance, 1.0f);
                SeparationForce += AwayFromNeighbor;
                SeparationCount++;
            }
            
            // Alignment
            AverageDirection += Directions[NeighborIdx];
            
            // Cohesion
            CenterOfMass += Positions[NeighborIdx];
        }
        
        // Force normalization
        if (SeparationCount > 0)
        {
            SeparationForce /= (float)SeparationCount;
            if (!SeparationForce.IsNearlyZero())
            {
                SeparationForce.Normalize();
            }
        }
        OutSeparationForces[i] = SeparationForce;
        
        const int32 NeighborCount = NeighborCache.Neighbors[i].Num();
        if (NeighborCount > 0)
        {
            // Alignment
            if (!AverageDirection.IsNearlyZero())
            {
                AverageDirection.Normalize();
            }
            OutAlignmentForces[i] = AverageDirection;
            
            // Cohesion
            CenterOfMass /= NeighborCount;
            FVector DirectionToCenter = CenterOfMass - Positions[i];
            if (!DirectionToCenter.IsNearlyZero())
            {
                DirectionToCenter.Normalize();
            }
            OutCohesionForces[i] = DirectionToCenter;
        }
    });
}

void UBoidSystem::CalculateBoundaryForces(TArray<FVector>& OutForces) const
{
    // Fast memory zeroing
    FMemory::Memzero(OutForces.GetData(), OutForces.Num() * sizeof(FVector));
    
    if (!OwnerManager)
    {
        return;
    }

    const FVector BoxOrigin = OwnerManager->SpawnVolume->GetComponentLocation();
    const FVector BoxExtent = OwnerManager->SpawnVolume->GetScaledBoxExtent();
    
    auto CalculateAxisForce = [](const float LocalPosition, const float AxisExtent, const float Margin, const float Strength, const float Exponent, const float EmerDist, const float EmerFactor) -> float
    {
        const float DistanceToPositiveWall = AxisExtent - LocalPosition;
        const float DistanceToNegativeWall = -AxisExtent - LocalPosition;
        float Force = 0.0f;
        
        if (DistanceToPositiveWall < Margin)
        {
            const float Factor = 1.0f - (DistanceToPositiveWall / Margin);
            const float ClampedFactor = FMath::Clamp(Factor, 0.0f, 1.0f);
            
            Force = -Strength * FMath::Pow(ClampedFactor, Exponent);
            
            if (DistanceToPositiveWall < EmerDist)
            {
                float EmergencyRatio = 1.0f - (DistanceToPositiveWall / EmerDist);
                Force *= (1.0f + EmerFactor * EmergencyRatio);
            }
        }
        else if (DistanceToNegativeWall > -Margin)
        {
            const float Factor = 1.0f - (-DistanceToNegativeWall / Margin);
            const float ClampedFactor = FMath::Clamp(Factor, 0.0f, 1.0f);
            
            Force = Strength * FMath::Pow(ClampedFactor, Exponent);
            
            if (-DistanceToNegativeWall < EmerDist)
            {
                float EmergencyRatio = 1.0f - (-DistanceToNegativeWall / EmerDist);
                Force *= (1.0f + EmerFactor * EmergencyRatio);
            }
        }
        
        return Force;
    };
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        constexpr float BoundaryMargin = 150.0f;
        constexpr float ForceStrength = 2.0f;
        constexpr float ForceExponent = 0.9f;
        constexpr float EmergencyDistance = 30.0f;
        constexpr float EmergencyFactor = 3.0f;

        const FVector LocalPos = Positions[i] - BoxOrigin;
        FVector BoundaryForce;
        
        BoundaryForce.X = CalculateAxisForce(LocalPos.X, BoxExtent.X, BoundaryMargin, ForceStrength, ForceExponent, EmergencyDistance, EmergencyFactor);
        BoundaryForce.Y = CalculateAxisForce(LocalPos.Y, BoxExtent.Y, BoundaryMargin, ForceStrength, ForceExponent, EmergencyDistance, EmergencyFactor);
        BoundaryForce.Z = CalculateAxisForce(LocalPos.Z, BoxExtent.Z, BoundaryMargin, ForceStrength, ForceExponent, EmergencyDistance, EmergencyFactor);
        
        OutForces[i] = BoundaryForce;
    }
}

void UBoidSystem::CalculateObstacleAvoidanceForces(TArray<FVector>& OutForces) const
{
    // Fast memory zeroing
    FMemory::Memzero(OutForces.GetData(), OutForces.Num() * sizeof(FVector));
    
    if (!bEnableObstacleAvoidance || !OwnerManager)
    {
        return;
    }

    const UWorld* World = OwnerManager->GetWorld();
    if (!World)
    {
        return;
    }
    
    ParallelFor(Positions.Num(), [&](int32 i)
    {
        FVector AvoidanceForce = FVector::ZeroVector;
        FVector BoidPosition = Positions[i];
        FVector BoidDirection = Directions[i];
        int32 HitCount = 0;
        
        for (const FRotator& Rotator : RaycastRotators)
        {
            FVector WorldDir = Rotator.RotateVector(BoidDirection);
            
            FHitResult HitResult;
            
            const bool bHit = World->LineTraceSingleByChannel(
                HitResult,
                BoidPosition,
                BoidPosition + WorldDir * ObstacleDetectionDistance,
                ECC_WorldStatic,
                GlobalObstacleQueryParams
            );
            
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
    });
}

void UBoidSystem::UpdatePositions(const float DeltaTime)
{
    ParallelFor(Positions.Num(), [&](int32 i)
    {
        const FVector Movement = Directions[i] * Velocity * DeltaTime;
        Positions[i] += Movement;
        
        if (BoidActors[i] && BoidActors[i]->BoidsManager)
        {
            Positions[i] = BoidActors[i]->BoidsManager->ConstrainPositionToBox(Positions[i]);
        }
    });

    constexpr float MinUpdateDistanceSquared = 1.0f;
    
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        if (BoidActors[i])
        {
            if (FVector::DistSquared(LastUpdatedPositions[i], Positions[i]) > MinUpdateDistanceSquared)
            {
                BoidActors[i]->SetActorLocationAndRotation(Positions[i], Directions[i].Rotation());
                LastUpdatedPositions[i] = Positions[i];
            }
        }
    }
}

void UBoidSystem::GenerateRaycastRotators()
{
    RaycastRotators.Empty();

    if (!bUseUniformDistribution)
    {
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
    }
    else
    {
        const float GoldenRatio = (1.0f + FMath::Sqrt(5.0f)) / 2.0f;
        
        for (int32 i = 0; i < NumberOfRaycasts; ++i)
        {
            // Vertical placement
            const float t = (float)i / NumberOfRaycasts;
            const float Theta = FMath::Acos(1.0f - 2.0f * t);
            
            // horizontal placement
            const float Phi = 2.0f * PI * GoldenRatio * i;

            const float X = FMath::Sin(Theta) * FMath::Cos(Phi);
            const float Y = FMath::Sin(Theta) * FMath::Sin(Phi);
            const float Z = FMath::Cos(Theta);
            
            FVector Direction(X, Y, Z);
            FRotator Rotator = Direction.Rotation();
            
            RaycastRotators.Add(Rotator);
        }
    }
    
    while (RaycastRotators.Num() > NumberOfRaycasts)
    {
        RaycastRotators.RemoveAt(RaycastRotators.Num() - 1);
    }
}

void UBoidSystem::SetUseUniformDistribution(bool bInUseUniformDistribution)
{
    if (bUseUniformDistribution != bInUseUniformDistribution)
    {
        bUseUniformDistribution = bInUseUniformDistribution;
        GenerateRaycastRotators();
    }
}

bool UBoidSystem::AreNeighbors(const int32 BoidA, const int32 BoidB, const float Radius) const
{
    if (BoidA == BoidB)
    {
        return false;
    }
    
    const float DistSquared = FVector::DistSquared(Positions[BoidA], Positions[BoidB]);
    return DistSquared <= Radius * Radius;
}

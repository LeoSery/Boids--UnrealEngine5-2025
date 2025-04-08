#include "BoidSystem.h"
#include "BoidsManager.h"
#include "Boid.h"

/**
 * @brief Default constructor
 * @details Initializes the raycast system with default values and generates initial raycast rotators
 */
UBoidSystem::UBoidSystem() : OwnerManager(nullptr)
{
    NumberOfRaycasts = 8;
    bUseUniformDistribution = false;
    GenerateRaycastRotators();
}

/**
 * @brief Initializes the boid system with the specified number of boids
 * @details Sets up all necessary arrays and data structures for simulation
 * 
 * @param NumBoids Number of boids to initialize
 * @param InitialPositions Starting positions for each boid
 * @param InitialDirections Starting directions for each boid
 */
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

/**
 * @brief Updates the entire boid simulation for one frame
 * @details Calculates all forces affecting each boid, combines them based on priorities,
 *          and updates positions and orientations
 * 
 * @param DeltaTime Time elapsed since the last frame in seconds
 */
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

        // Force prioritization system:
        // 1. Obstacle avoidance (highest priority - safety critical)
        // 2. Boundary forces (second priority - containment)
        // 3. Flocking behaviors (lowest priority - only applied when safe)
        if (!CachedObstacleAvoidanceForces[i].IsNearlyZero())
        {
            // Obstacle avoidance force
            TotalForce = CachedObstacleAvoidanceForces[i] * ObstacleAvoidanceWeight;
        }
        else if (!CachedBoundaryForces[i].IsNearlyZero())
        {
            // Boundary avoidance force
            TotalForce = CachedBoundaryForces[i] * BoundaryWeight;
        }
        else
        {
            // Separation force
            if (!CachedSeparationForces[i].IsNearlyZero())
                TotalForce += CachedSeparationForces[i] * SeparationWeight;
            
            // Alignment force
            if (!CachedAlignmentForces[i].IsNearlyZero())
                TotalForce += CachedAlignmentForces[i] * AlignmentWeight;
            
            // Cohesion force
            if (!CachedCohesionForces[i].IsNearlyZero())
                TotalForce += CachedCohesionForces[i] * CohesionWeight;
        }
        
        if (TotalForce.IsNearlyZero())
            continue;
        
        TotalForce.Normalize();
        
        // smoothly apply the new direction
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

/**
 * @brief Sets all behavior parameters for the boid simulation
 * @details Updates all parameters related to flocking behavior and recalculates dependent values
 * 
 * @param InSeparationWeight Weight of the separation force
 * @param InAlignmentWeight Weight of the alignment force
 * @param InCohesionWeight Weight of the cohesion force
 * @param InSeparationRadius Distance at which separation begins to take effect
 * @param InPerceptionRadius Maximum distance at which boids can detect each other
 * @param InBoundaryWeight Weight of boundary avoidance forces
 * @param InVelocity Base movement speed of boids
 * @param InFieldOfViewAngle Angle of visibility cone in degrees
 */
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

/**
 * @brief Sets all obstacle avoidance parameters
 * @details Updates parameters controlling how boids detect and avoid obstacles
 * 
 * @param InObstacleAvoidanceWeight Weight of obstacle avoidance forces
 * @param InObstacleDetectionDistance Maximum distance for obstacle detection
 * @param InNumberOfRaycasts Number of rays used for obstacle detection
 * @param bInEnableObstacleAvoidance Whether obstacle avoidance is active
 */
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

/**
 * @brief Generates the set of rotators used for obstacle detection raycasts
 * @details Creates either a uniform distribution using golden ratio or a predefined pattern
 *          based on the bUseUniformDistribution setting
 */
void UBoidSystem::GenerateRaycastRotators()
{
    RaycastRotators.Empty();

    if (!bUseUniformDistribution)
    {
        RaycastRotators.Add(FRotator::ZeroRotator);

        constexpr float YawAngle = 30.0f;    // Horizontal angle
        constexpr float PitchAngle = 30.0f;  // Vertical angle
    
        RaycastRotators.Add(FRotator(0, YawAngle, 0));       // Right
        RaycastRotators.Add(FRotator(0, -YawAngle, 0));      // Left
        RaycastRotators.Add(FRotator(PitchAngle, 0, 0));     // Top
        RaycastRotators.Add(FRotator(-PitchAngle, 0, 0));    // Bottom
    
        if (NumberOfRaycasts > 5)
        {
            RaycastRotators.Add(FRotator(PitchAngle, YawAngle, 0));     // Top right
            RaycastRotators.Add(FRotator(PitchAngle, -YawAngle, 0));    // Top left
            RaycastRotators.Add(FRotator(-PitchAngle, YawAngle, 0));    // Bottom right
            RaycastRotators.Add(FRotator(-PitchAngle, -YawAngle, 0));   // Bottom left
        }
    
        if (NumberOfRaycasts > 9)
        {
            constexpr float HalfYaw = YawAngle * 0.5f;
            constexpr float HalfPitch = PitchAngle * 0.5f;
        
            RaycastRotators.Add(FRotator(0, HalfYaw, 0));         // Half right
            RaycastRotators.Add(FRotator(0, -HalfYaw, 0));        // Half left
            RaycastRotators.Add(FRotator(HalfPitch, 0, 0));       // Half top
            RaycastRotators.Add(FRotator(-HalfPitch, 0, 0));      // Half bottom
        }
    }
    else
    {
        // Fibonacci sphere algorithm for uniform point distribution on a sphere
        // Uses the golden ratio (φ ≈ 1.618) to create optimal spacing between points
        const float GoldenRatio = (1.0f + FMath::Sqrt(5.0f)) / 2.0f;
        
        for (int32 i = 0; i < NumberOfRaycasts; ++i)
        {
            // Vertical placement - evenly distributes points from top to bottom
            // Maps i/N to an angle in [0,π] with improved uniformity
            const float t = static_cast<float>(i) / NumberOfRaycasts;
            const float Theta = FMath::Acos(1.0f - 2.0f * t);
            
            // Horizontal placement - uses golden ratio to spiral around the sphere
            // This ensures points never align, maximizing coverage
            const float Phi = 2.0f * PI * GoldenRatio * i;

            const float X = FMath::Sin(Theta) * FMath::Cos(Phi);
            const float Y = FMath::Sin(Theta) * FMath::Sin(Phi);
            const float Z = FMath::Cos(Theta);

            // Convert from spherical to Cartesian coordinates
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

/**
 * @brief Sets whether to use uniform distribution for raycasts
 * @details When changed, automatically regenerates raycast rotators with the new distribution
 * 
 * @param bInUseUniformDistribution Whether to use uniform distribution (true) or predefined pattern (false)
 */
void UBoidSystem::SetUseUniformDistribution(const bool bInUseUniformDistribution)
{
    if (bUseUniformDistribution != bInUseUniformDistribution)
    {
        bUseUniformDistribution = bInUseUniformDistribution;
        GenerateRaycastRotators();
    }
}

/**
 * @brief Calculates separation, alignment and cohesion forces for all boids
 * @details Multithreaded calculation of the three core flocking forces for each boid
 *          based on its neighbors
 * 
 * @param OutSeparationForces Array to store calculated separation forces
 * @param OutAlignmentForces Array to store calculated alignment forces
 * @param OutCohesionForces Array to store calculated cohesion forces
 */
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
    
    ParallelFor(NumBoids, [&](const int32 i)
    {
        if (NeighborCache.Neighbors[i].Num() == 0)
        {
            return;
        }
        
        int32 SeparationCount = 0;
        FVector SeparationForce = FVector::ZeroVector;
        FVector AverageDirection = FVector::ZeroVector;
        FVector CenterOfMass = FVector::ZeroVector;

        // Calculation of the three core flocking forces:
        // Separation - steer away from nearby neighbors
        // Alignment - steer towards average heading of neighbors
        // Cohesion - steer towards center of local flock
        for (const int32 NeighborIdx : NeighborCache.Neighbors[i])
        {
            // Separation: Push away from neighbors that are too close
            // Force increases as distance decreases
            const float Distance = FVector::Dist(Positions[i], Positions[NeighborIdx]);
            if (Distance < SeparationRadius && Distance > 0)
            {
                FVector AwayFromNeighbor = Positions[i] - Positions[NeighborIdx];
                AwayFromNeighbor.Normalize();
                AwayFromNeighbor *= SeparationRadius / FMath::Max(Distance, 1.0f);
                SeparationForce += AwayFromNeighbor;
                SeparationCount++;
            }
            
            // Alignment: Steer in same direction as neighbors
            // Simply accumulate all neighbor directions for later averaging
            AverageDirection += Directions[NeighborIdx];
            
            // Cohesion: Steer toward center of local flock
            // Accumulate positions for calculating center of mass
            CenterOfMass += Positions[NeighborIdx];
        }
        
        // Force normalization
        if (SeparationCount > 0)
        {
            SeparationForce /= static_cast<float>(SeparationCount);
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
            // Finalize alignment: Normalize the sum of directions to get average heading
            if (!AverageDirection.IsNearlyZero())
            {
                AverageDirection.Normalize();
            }
            OutAlignmentForces[i] = AverageDirection;
            
            // Cohesion
            // Finalize cohesion: Calculate center of mass, then direction toward it
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

/**
 * @brief Calculates forces that keep boids within the boundary box
 * @details Applies progressively stronger forces as boids approach boundaries,
 *          with emergency stronger forces very close to edges
 * 
 * @param OutForces Array to store calculated boundary forces
 */
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

    // Lambda function that calculates boundary repulsion force for a single axis
    // Creates a smooth force field that increases as boids approach boundaries
    // Includes an "emergency zone" with exponentially stronger forces when very close to edges
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
                const float EmergencyRatio = 1.0f - (-DistanceToNegativeWall / EmerDist);
                Force *= (1.0f + EmerFactor * EmergencyRatio);
            }
        }
        
        return Force;
    };

    // Calculate boundary forces with dynamic adjustment based on speed
    // Faster boids receive stronger forces and earlier warnings to prevent breaches
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        constexpr float ForceExponent = 0.9f;
        constexpr float EmergencyFactor = 3.0f;
        constexpr float BoundaryMargin = 150.0f;
        constexpr float ForceStrength = 2.0f;
        constexpr float EmergencyDistance = 30.0f;
        
        const float CurrentSpeed = Directions[i].Size();
        const float SpeedRatio = CurrentSpeed / Velocity;

        const float AdjustedBoundaryMargin = BoundaryMargin * (1.0f + SpeedRatio);
        const float AdjustedForceStrength = ForceStrength * (1.0f + SpeedRatio);
        const float AdjustedEmergencyDistance = EmergencyDistance * (1.0f + SpeedRatio);

        const FVector LocalPos = Positions[i] - BoxOrigin;
        FVector BoundaryForce;
        
        BoundaryForce.X = CalculateAxisForce(LocalPos.X, BoxExtent.X, AdjustedBoundaryMargin, AdjustedForceStrength, ForceExponent, AdjustedEmergencyDistance, EmergencyFactor);
        BoundaryForce.Y = CalculateAxisForce(LocalPos.Y, BoxExtent.Y, AdjustedBoundaryMargin, AdjustedForceStrength, ForceExponent, AdjustedEmergencyDistance, EmergencyFactor);
        BoundaryForce.Z = CalculateAxisForce(LocalPos.Z, BoxExtent.Z, AdjustedBoundaryMargin, AdjustedForceStrength, ForceExponent, AdjustedEmergencyDistance, EmergencyFactor);
        
        OutForces[i] = BoundaryForce;
    }
}

/**
 * @brief Calculates forces to steer boids away from obstacles
 * @details Uses raycasts to detect obstacles and generates avoidance forces
 *          based on distance and approach angle
 * 
 * @param OutForces Array to store calculated obstacle avoidance forces
 */
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
    
    const float ObstacleMargin = ObstacleDetectionDistance;
    constexpr float ForceStrength = 3.0f;
    constexpr float ForceExponent = 0.9f;
    constexpr float EmergencyDistance = 100.0f;
    constexpr float EmergencyFactor = 5.0f;

    // Multi-threaded obstacle avoidance calculation
    // Casts rays in multiple directions and generates steering forces away from detected obstacles
    // Force strength is dynamically adjusted based on boid speed and distance to obstacle
    ParallelFor(Positions.Num(), [&](const int32 i)
    {
        const float CurrentSpeed = Directions[i].Size();
        const float SpeedRatio = CurrentSpeed / Velocity;
        
        const float AdjustedDetectionDistance = ObstacleMargin * (1.0f + SpeedRatio * 1.5f);
        
        FVector AvoidanceForce = FVector::ZeroVector;
        const FVector BoidPosition = Positions[i];
        const FVector BoidDirection = Directions[i];
        int32 HitCount = 0;

        // Cast rays in different directions around boid's forward vector
        // Each ray checks for obstacles within detection distance
        for (const FRotator& Rotator : RaycastRotators)
        {
            FVector WorldDir = Rotator.RotateVector(BoidDirection);
            
            FHitResult HitResult;
            
            const bool bHit = World->LineTraceSingleByChannel(
                HitResult,
                BoidPosition,
                BoidPosition + WorldDir * AdjustedDetectionDistance,
                ECC_WorldStatic,
                GlobalObstacleQueryParams
            );

            // When obstacle detected, calculate repulsion force based on distance
            // Closer obstacles generate exponentially stronger forces
            if (bHit)
            {
                const float Distance = HitResult.Distance;
                
                const float Factor = 1.0f - Distance / AdjustedDetectionDistance;
                float ClampedFactor = FMath::Clamp(Factor, 0.0f, 1.0f);
                
                float Force = ForceStrength * FMath::Pow(ClampedFactor, ForceExponent);
                
                if (Distance < EmergencyDistance * (1.0f + SpeedRatio))
                {
                    const float EmergencyRatio = 1.0f - Distance / (EmergencyDistance * (1.0f + SpeedRatio));
                    Force *= 1.0f + EmergencyFactor * EmergencyRatio;
                }

                Force *= 1.0f + SpeedRatio * 1.5f;
                
                FVector AwayFromObstacle = -WorldDir * Force;
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

/**
 * @brief Updates the positions of all boids based on their current direction and velocity
 * @details Multithreaded update of positions with boundary constraints
 * 
 * @param DeltaTime Time elapsed since the last frame in seconds
 */
void UBoidSystem::UpdatePositions(const float DeltaTime)
{
    ParallelFor(Positions.Num(), [&](const int32 i)
    {
        const FVector Movement = Directions[i] * Velocity * DeltaTime;
        Positions[i] += Movement;
        
        if (BoidActors[i] && BoidActors[i]->BoidsManager)
        {
            Positions[i] = BoidActors[i]->BoidsManager->ConstrainPositionToBox(Positions[i]);
        }
    });
}

/**
 * @brief Finds all neighbors for each boid
 * @details Multithreaded calculation that determines which boids are visible
 *          to each other based on distance and field of view
 */
void UBoidSystem::FindAllNeighbors()
{
    NeighborCache.Clear();
    
    const float PerceptionRadiusSq = PerceptionRadius * PerceptionRadius;
    const int32 NumBoids = Positions.Num();

    // Temporary storage to allow parallel computation without race conditions
    TArray<TArray<int32>> TempNeighbors;
    TempNeighbors.SetNum(NumBoids);

    // Multi-threaded neighbor detection for performance optimization
    ParallelFor(NumBoids, [&](const int32 i)
    {
        TArray<int32>& MyNeighbors = TempNeighbors[i];
        const FVector& MyPosition = Positions[i];
        const FVector& MyDirection = Directions[i];
        
        for (int32 j = 0; j < NumBoids; ++j)
        {
            if (i == j)
            {
                continue;
            }
            
            const float DistSquared = FVector::DistSquared(MyPosition, Positions[j]);
            
            if (DistSquared <= PerceptionRadiusSq)
            {
                // Field of view check using dot product
                // Only boids within the vision cone are considered neighbor
                const FVector DirectionToOther = (Positions[j] - MyPosition).GetSafeNormal();
                const float DotProduct = FVector::DotProduct(MyDirection, DirectionToOther);
                
                if (DotProduct >= FOVDotProductThreshold)
                {
                    MyNeighbors.Add(j);
                }
            }
        }
    });

    // Transfer results from temporary arrays to persistent cache
    for (int32 i = 0; i < NumBoids; ++i)
    {
        NeighborCache.Neighbors[i] = MoveTemp(TempNeighbors[i]);
    }
    
    int32 MaxNeighborsThisFrame = 0;
    for (int32 i = 0; i < NumBoids; ++i)
    {
        MaxNeighborsThisFrame = FMath::Max(MaxNeighborsThisFrame, NeighborCache.Neighbors[i].Num());
    }
    
    LastFrameMaxNeighbors = FMath::Max(1, MaxNeighborsThisFrame + (MaxNeighborsThisFrame / 4));
}

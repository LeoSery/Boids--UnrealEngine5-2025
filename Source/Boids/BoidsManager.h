#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "BoidsManager.generated.h"

class ABoid;

UCLASS()
class BOIDS_API ABoidsManager : public AActor
{
	GENERATED_BODY()

public:
	ABoidsManager();

	virtual void Tick(float DeltaTime) override;
	
	UPROPERTY(EditAnywhere, Category = "Boids|Spawning")
	int32 NumberOfBoids;
	
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Boids|Spawning")
	UBoxComponent* SpawnVolume;
	
	UPROPERTY(EditAnywhere, Category = "Boids|Spawning")
	TSubclassOf<ABoid> BoidPrefab;
	
	void SpawnBoids();
	
	TArray<ABoid*> GetNearbyBoids(ABoid* Boid, float Radius) const;

	FVector ConstrainPositionToBox(const FVector& Position);
	
protected:
	virtual void BeginPlay() override;

	UPROPERTY()
	TArray<ABoid*> Boids;
};

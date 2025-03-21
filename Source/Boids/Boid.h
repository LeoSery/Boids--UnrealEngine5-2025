#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Boid.generated.h"

class ABoidsManager;
class UBoidSystem;

UCLASS()
class BOIDS_API ABoid : public AActor
{
	GENERATED_BODY()

public:
	ABoid();
	
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boids|Settings")
	UStaticMeshComponent* Mesh;
	
	int32 BoidIndex = -1;
	
	UPROPERTY()
	ABoidsManager* BoidsManager = nullptr;
	
	UPROPERTY()
	UBoidSystem* BoidSystem = nullptr;
	
protected:

	UPROPERTY()
	USceneComponent* Root;
};

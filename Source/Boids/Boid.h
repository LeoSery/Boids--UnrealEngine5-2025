#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Boid.generated.h"

UCLASS()
class BOIDS_API ABoid : public AActor
{
	GENERATED_BODY()

public:
	ABoid();
	
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, Category = "Settings")
	float Velocity;

	UPROPERTY(EditAnywhere, Category = "Settings")
	FVector Direction;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Settings")
	UStaticMeshComponent* Mesh;

	UPROPERTY()
	USceneComponent* Root;
	
protected:
	virtual void BeginPlay() override;
};

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Boid.generated.h"

//////// FORWARD DECLARATION ////////
class ABoidsManager;
class UBoidSystem;

//////// CLASS ////////
/// Individual boid actor representing a single member of the flock
UCLASS()
class BOIDS_API ABoid : public AActor
{
	GENERATED_BODY()

public:
	//////// UNREAL LIFECYCLE ////////
	ABoid();
	
	virtual void BeginPlay() override;
	
	virtual void Tick(const float DeltaTime) override;

	//////// COMPONENTS ////////
	/// Mesh
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Boids|Settings")
	UStaticMeshComponent* Mesh;

	//////// PROPERTIES ////////
	/// General
	int32 BoidIndex = -1;

	/// Managers
	UPROPERTY()
	ABoidsManager* BoidsManager = nullptr;
	
	UPROPERTY()
	UBoidSystem* BoidSystem = nullptr;

	/// Debug
	UPROPERTY(EditAnywhere, Category = "Boids|Debug")
	bool bDebugRaycasts = false;
	
protected:

	//////// COMPONENTS ////////
	/// Root
	UPROPERTY()
	USceneComponent* Root;

	//////// METHODS ////////
	void DebugDrawRaycasts() const;
};

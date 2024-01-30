// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "FighterJetAnimInstance.generated.h"

/**
 * 
 */
UCLASS()
class FLIGHTJET_API UFighterJetAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

public:
	virtual void NativeInitializeAnimation() override;
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;

private:
	UPROPERTY(BlueprintReadOnly, Category = FlightComponent, meta = (AllowPrivateAccess = "true"))
	class UFlightPhysicsComponent* FlightComponent;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<float> SuspensionDisplacements;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<FRotator> TyresRotation;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<float> StablizersPitch;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<float> AileronsPitch;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<float> FlapsPitch;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<float> RuddersYaw;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	bool bIsWheelsRetreated;
	
public:
	FORCEINLINE void SetFlightComponent(UFlightPhysicsComponent* AnimInsPtr) { FlightComponent = AnimInsPtr; }
	FORCEINLINE UFlightPhysicsComponent* GetFlightComponent() const { return FlightComponent; }
};

// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AirEnvironmentManager.generated.h"

UCLASS()
class FLIGHTJET_API AAirEnvironmentManager : public AActor
{
	GENERATED_BODY()
	
public:	
	AAirEnvironmentManager();

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

private:
	FRandomStream RandomStream;

	float TimeFactor = 1.0f;

	float CalculateAirDensity(float Altitude, float NoiseValue);

	FVector CalculateWindSpeed(float Altitude, float NoiseValue);

	float GenerateNoiseBasedValue(const FVector& Location);

public:	
	UPROPERTY(EditAnywhere, Category = "Custom Parameters")
	float BaseAltitude = 0.0f;
	
	/** Get the air density of the given altitude */
	UFUNCTION(BlueprintCallable)
	float GetAirDensityAtLocation(const FVector& Location);

	/** Get the wind speed of the given altitude */
	UFUNCTION(BlueprintCallable)
	FVector GetWindSpeedAtLocation(const FVector& Location);

};

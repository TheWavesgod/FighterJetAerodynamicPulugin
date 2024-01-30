// Fill out your copyright notice in the Description page of Project Settings.


#include "AirEnvironmentManager.h"

// Sets default values
AAirEnvironmentManager::AAirEnvironmentManager()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
}

// Called when the game starts or when spawned
void AAirEnvironmentManager::BeginPlay()
{
	Super::BeginPlay();
	
}

void AAirEnvironmentManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

float AAirEnvironmentManager::CalculateAirDensity(float Altitude, float NoiseValue)
{
	const float StandardAirDensityAtSeaLevel = 1.225f; 

	const float AtmosphericScaleHeight = 8500.0f; 

	float BaseAirDensity = StandardAirDensityAtSeaLevel * FMath::Exp(-Altitude / AtmosphericScaleHeight);

	const float NoiseImpactFactor = 0.1f;

	float FinalAirDensity = BaseAirDensity + (NoiseValue * NoiseImpactFactor * BaseAirDensity);

	return FMath::Max(FinalAirDensity, 0.0f);
}

FVector AAirEnvironmentManager::CalculateWindSpeed(float Altitude, float NoiseValue)
{
	return FVector();
}

float AAirEnvironmentManager::GetAirDensityAtLocation(const FVector& Location)
{
	float Altitude = (Location.Z - BaseAltitude) / 100.0f;
	float NoiseValue = GenerateNoiseBasedValue(Location);
	return CalculateAirDensity(Altitude, NoiseValue);
}

FVector AAirEnvironmentManager::GetWindSpeedAtLocation(const FVector& Location)
{
	float Altitude = (Location.Z - BaseAltitude) / 100.0f;
	float NoiseValue = GenerateNoiseBasedValue(Location);
	CalculateWindSpeed(Altitude, NoiseValue);
	return FVector::ZeroVector;
}

float AAirEnvironmentManager::GenerateNoiseBasedValue(const FVector& Location)
{
	float NoiseValue = FMath::PerlinNoise3D(FVector(Location.X, Location.Y, Location.Z + TimeFactor));
	return NoiseValue;
}
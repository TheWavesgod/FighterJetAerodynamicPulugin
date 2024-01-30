// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameState.h"
#include "FlightTestGameState.generated.h"

/**
 * 
 */
UCLASS()
class FIGHTERPLUGIN_API AFlightTestGameState : public AGameState
{
	GENERATED_BODY()

public:
	AFlightTestGameState();

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Enviroment")
	class AAirEnvironmentManager* AirEnvironmentManager;

protected:
	
private:
	
};

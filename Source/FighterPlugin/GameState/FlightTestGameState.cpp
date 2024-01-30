// Fill out your copyright notice in the Description page of Project Settings.


#include "FlightTestGameState.h"
#include "FlightJet/Public/AirEnvironmentManager.h"

AFlightTestGameState::AFlightTestGameState()
{
	AirEnvironmentManager = CreateDefaultSubobject<AAirEnvironmentManager>(TEXT("AirEnvironmentManager"));
}

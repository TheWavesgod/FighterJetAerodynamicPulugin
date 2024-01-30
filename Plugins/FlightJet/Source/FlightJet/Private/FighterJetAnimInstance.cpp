// Fill out your copyright notice in the Description page of Project Settings.


#include "FighterJetAnimInstance.h"
#include "FlightPhysicsComponent.h"

void UFighterJetAnimInstance::NativeInitializeAnimation()
{
	Super::NativeInitializeAnimation();
}

void UFighterJetAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	Super::NativeUpdateAnimation(DeltaSeconds);

	if (!FlightComponent) return;

	for (int i = 0; i < FlightComponent->WheelAnimVaribles.Num(); ++i)
	{
		if (SuspensionDisplacements.Num() < i + 1)
		{
			SuspensionDisplacements.Emplace();
		}
		SuspensionDisplacements[i] = FlightComponent->WheelAnimVaribles[i].SuspensionDisplacement;

		if (TyresRotation.Num() < i + 1)
		{
			TyresRotation.Emplace();
		}
		TyresRotation[i] = FlightComponent->WheelAnimVaribles[i].WheelRotation;
	}
	for (int i = 0; i < FlightComponent->StablizersPitch.Num(); ++i)
	{
		if (StablizersPitch.Num() < i + 1)
		{
			StablizersPitch.Emplace();
		}
		StablizersPitch[i] = FlightComponent->StablizersPitch[i];
	}
	for (int i = 0; i < FlightComponent->AileronsPitch.Num(); ++i)
	{
		if (AileronsPitch.Num() < i + 1)
		{
			AileronsPitch.Emplace();
		}
		AileronsPitch[i] = FlightComponent->AileronsPitch[i];
	}
	for (int i = 0; i < FlightComponent->FlapsPitch.Num(); ++i)
	{
		if (FlapsPitch.Num() < i + 1)
		{
			FlapsPitch.Emplace();
		}
		FlapsPitch[i] = FlightComponent->FlapsPitch[i];
	}
	for (int i = 0; i < FlightComponent->RuddersYaw.Num(); ++i)
	{
		if (RuddersYaw.Num() < i + 1)
		{
			RuddersYaw.Emplace();
		}
		RuddersYaw[i] = FlightComponent->RuddersYaw[i];
	}
}

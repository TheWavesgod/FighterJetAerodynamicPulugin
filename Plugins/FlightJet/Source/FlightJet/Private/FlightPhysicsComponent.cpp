// Fill out your copyright notice in the Description page of Project Settings.


#include "FlightPhysicsComponent.h"
#include "Fighter.h"
#include "Components/SkeletalMeshComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "FighterJetAnimInstance.h"
#include "AirEnvironmentManager.h"

// Sets default values for this component's properties
UFlightPhysicsComponent::UFlightPhysicsComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	
	Fighter = GetOwner<AFighter>();
	if (Fighter)
	{
		JetMesh = Fighter->GetMesh();
		JetMesh->SetSimulatePhysics(true);
		JetMesh->SetEnableGravity(true);
	}
}

// Called when the game starts
void UFlightPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();		
	
	if (JetMesh)
	{
		JetMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		JetMesh->SetCollisionObjectType(ECC_Pawn);
		JetMesh->SetMassOverrideInKg(FName("cog_jnt"), JetEmptyWeight);
		FVector WorldLocationOfMassCenter = JetMesh->GetCenterOfMass();
		FVector LocalLocationOfMassCenter = JetMesh->GetComponentTransform().InverseTransformPosition(WorldLocationOfMassCenter);
		JetMesh->SetCenterOfMass(CenterOfMass - LocalLocationOfMassCenter, FName("cog_jnt"));
	}
}

// Called every frame
void UFlightPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	InitializeAnimationInstance();

	DebugEveryTick();

	JetParametersTick(DeltaTime);

	FlyingControlTick(DeltaTime);

	WheelsForceTick(DeltaTime);

	ThrusterForceTick(DeltaTime);
	 
	AerodynamicForceTick(DeltaTime);
}

void UFlightPhysicsComponent::DebugEveryTick()
{
	AddDebugMessageOnScreen(0.0f, FColor::Blue, FString::Printf(TEXT("Jet's Weight: %d kg"), FMath::RoundToInt32(JetMesh->GetMass())));
	for (auto i : WheelsForcesToAdd)
	{
		AddDebugMessageOnScreen(0.0f, FColor::Green, FString::Printf(TEXT("WheelForce: %d"), FMath::RoundToInt32(i.Length())));
	}
	if (JetMesh)
	{
		DrawDebugSphere(GetWorld(), JetMesh->GetCenterOfMass(), 30.0f, 8, FColor::Red, false, 0.0f);
	}
	for (int i = 0; i < CurrentThrusters.Num(); ++i)
	{
		AddDebugMessageOnScreen(0.0f, FColor::Red, FString::Printf(TEXT("Thruster %d: %d"), i, FMath::RoundToInt32(CurrentThrusters[i])));
	}
	DrawDebugLine(GetWorld(), JetMesh->GetComponentLocation(), JetMesh->GetComponentLocation() + JetMesh->GetPhysicsLinearVelocity(), FColor::Red, false, 0.0f);
}

void UFlightPhysicsComponent::AddDebugMessageOnScreen(const float DisplayTime, const FColor Color, const FString DiplayString)
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1,
			DisplayTime,
			Color,
			DiplayString
		);
	}
}

void UFlightPhysicsComponent::JetParametersTick(float DeltaTime)
{
	LastFrameJetMeshVelocity = JetMeshVelocity;
	JetMeshVelocity = JetMesh->GetPhysicsLinearVelocity();
	JetMeshAcceleration = (JetMeshVelocity - LastFrameJetMeshVelocity) / DeltaTime;

	JetMeshAngularVelocityInRadians = JetMesh->GetPhysicsAngularVelocityInRadians();

	GForce = CalculateCurrentGForce(DeltaTime);

	FVector JetForwardVelocity = JetMeshVelocity.ProjectOnTo(JetMesh->GetForwardVector());
	GroundSpeed = JetForwardVelocity.Size() * 0.036;

	FVector RightPlaneVelocity = FVector::VectorPlaneProject(JetMeshVelocity, JetMesh->GetRightVector());
	if (RightPlaneVelocity.Size() > 100.0f)
	{
		FVector RightPlaneVelocityDir = RightPlaneVelocity;
		RightPlaneVelocityDir.Normalize();
		float SignValue = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(RightPlaneVelocity, JetMesh->GetUpVector()))) < 90.0f ? -1.0f : 1.0f;
		AngleOfAttack = SignValue * FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(RightPlaneVelocityDir, JetMesh->GetForwardVector())));
	}
	else
	{
		AngleOfAttack = 0.0f;
	}
}

float UFlightPhysicsComponent::CalculateCurrentGForce(float DeltaTime)
{
	return JetMeshAcceleration.Dot(JetMesh->GetUpVector()) / 981.0f;
}

void UFlightPhysicsComponent::WheelsForceTick(float DeltaTime)
{	
	if (!JetMesh) return; 

	WheelsRaycastAndCache();
	WheelsCalculateForces(DeltaTime);

	for (int i = 0; i < WheelsForcesToAdd.Num(); ++i)
	{
		JetMesh->AddForceAtLocation(
			WheelsForcesToAdd[i] * 100.0f,
			WheelsCacheValues[i].ImpactLocation
		);

		//DrawDebugLine(
		//	GetWorld(),
		//	WheelsCacheValues[i].ImpactLocation,
		//	WheelsCacheValues[i].ImpactLocation + WheelsForcesToAdd[i] / 1000.0f,
		//	FColor::Green,
		//	false,
		//	0.0f
		//);
	}
}

void UFlightPhysicsComponent::WheelsRaycastAndCache()
{
	UWorld* World = GetWorld();
	if (World)
	{
		for (int i = 0; i < JetTyres.Num(); ++i)
		{
			FVector CurrentWheelLocation = JetMesh->GetSocketLocation(JetTyres[i].WheelBoneName);
			float WheelRadius = JetTyres[i].WheelSettings.WheelRadius;
			FVector ChassisAxis = Fighter->GetActorTransform().TransformVector(JetTyres[i].ChassisSettings.SuspensionAxis);
			float ChassisMaxRaise = JetTyres[i].ChassisSettings.SuspensionMaxRaise;
			float ChassisMaxDrop = JetTyres[i].ChassisSettings.SuspensionMaxDrop;

			FVector Start = CurrentWheelLocation - ChassisAxis * (ChassisMaxRaise + WheelRayOffset);
			FVector End = CurrentWheelLocation + ChassisAxis * (ChassisMaxDrop + WheelRadius);
			FHitResult WheelHitResult;
			FCollisionQueryParams QueryParams = FCollisionQueryParams::DefaultQueryParam;
			QueryParams.AddIgnoredActor(Fighter);
			QueryParams.AddIgnoredComponent(JetMesh);
			QueryParams.bReturnPhysicalMaterial = true;

			World->LineTraceSingleByChannel(
				WheelHitResult,
				Start,
				End,
				ECollisionChannel::ECC_Pawn,
				QueryParams
			);

			if (WheelsCacheValues.Num() < i + 1)
			{
				FWheelCacheVaribles temp;
				WheelsCacheValues.Emplace(temp);
			}

			WheelsCacheValues[i].bIsHit = WheelHitResult.bBlockingHit;
			WheelsCacheValues[i].LastFrameImpactLocation = WheelsCacheValues[i].ImpactLocation;
			WheelsCacheValues[i].ImpactLocation = WheelHitResult.bBlockingHit ? WheelHitResult.ImpactPoint : FVector();
			WheelsCacheValues[i].LastFrameDistance = WheelsCacheValues[i].Distance;
			WheelsCacheValues[i].Distance = WheelHitResult.bBlockingHit ? (WheelHitResult.TraceStart - WheelHitResult.Location).Size() : (Start - End).Size();
			WheelsCacheValues[i].Normal = WheelHitResult.ImpactNormal;
			WheelsCacheValues[i].SurfaceMatrial = WheelHitResult.PhysMaterial.Get();
		}
	}
}

void UFlightPhysicsComponent::WheelsCalculateForces(float DeltaTime)
{
	for (int i = 0; i < WheelsCacheValues.Num(); ++i)
	{
		if (WheelsForcesToAdd.Num() < i + 1)
		{
			WheelsForcesToAdd.Emplace(FVector::ZeroVector);
		}
		if (WheelAnimVaribles.Num() < i + 1)
		{
			WheelAnimVaribles.Emplace();
		}

		if (!WheelsCacheValues[i].bIsHit)
		{	
			WheelsForcesToAdd[i] = FVector::ZeroVector;
			continue;
		}
		
		/** 
		 * Chassis Calculate 
		 */
		float SpringWholeLength = JetTyres[i].ChassisSettings.SuspensionMaxRaise + JetTyres[i].ChassisSettings.SuspensionMaxDrop + JetTyres[i].ChassisSettings.SpringPreLoadLength;
		float SpringCompressLength = WheelsCacheValues[i].Distance - JetTyres[i].WheelSettings.WheelRadius - WheelRayOffset;
		float SpringForceLength = SpringWholeLength - SpringCompressLength;
		float SpringForce = SpringForceLength * JetTyres[i].ChassisSettings.SpringRate * 0.01f;

		float SuspensionDistanceDisplacementVel = (WheelsCacheValues[i].LastFrameDistance - WheelsCacheValues[i].Distance) / DeltaTime;
		float SuspensionDamping = FMath::Abs(SuspensionDistanceDisplacementVel) > WheelStaticThreshold ? SuspensionDistanceDisplacementVel : 0.0f;
		float SuspensionDampingForce = JetTyres[i].ChassisSettings.SuspensionDampingRatio * SuspensionDamping;
		SuspensionDampingForce = FMath::Abs(SuspensionDampingForce) > SpringForce ? 0.0f : SuspensionDampingForce;

		//float ActualChassisForce = SpringCompressLength < 0 ? 0 : SpringForce + SuspensionDampingForce;
		float ActualChassisForce = SpringForce + SuspensionDampingForce;

		FVector SuspensionWorldAxis = Fighter->GetActorTransform().TransformVector(JetTyres[i].ChassisSettings.SuspensionAxis);
		FVector SuspensionForce = ActualChassisForce * (-SuspensionWorldAxis);

		WheelsForcesToAdd[i] = SuspensionForce;

		WheelAnimVaribles[i].SuspensionDisplacement = SpringForceLength - JetTyres[i].ChassisSettings.SpringPreLoadLength - JetTyres[i].ChassisSettings.SuspensionMaxDrop;

		/**
		 * FrictionCalculate
		 */
		FVector WheelVelocity = (WheelsCacheValues[i].ImpactLocation - WheelsCacheValues[i].LastFrameImpactLocation) / DeltaTime;
		FVector WheelPlaneVelocity = FVector::VectorPlaneProject(WheelVelocity, WheelsCacheValues[i].Normal);

		float SurfaceFrictionRatio = WheelsCacheValues[i].SurfaceMatrial ? WheelsCacheValues[i].SurfaceMatrial->Friction : 0.7;
		float WheelFrictionRatio = JetTyres[i].WheelSettings.WheelFrictionRatio;
		// Friction Ratio Combine mode = Average
		float FrictionRatio = (SurfaceFrictionRatio + WheelFrictionRatio) / 2.0f;
		float StaticFrictionRatio = FrictionRatio + 0.1;
		// Suspension Force resolusion
		FVector NormalSuspensionForce = SuspensionForce.ProjectOnToNormal(WheelsCacheValues[i].Normal);
		FVector PlaneSuspensionForce = FVector::VectorPlaneProject(SuspensionForce, WheelsCacheValues[i].Normal);
		// Calculate the wheels' forward and right direction
		FVector WheelForwardDir = JetMesh->GetForwardVector();
		FVector WheelRightDir = JetMesh->GetRightVector();
		if (JetTyres[i].WheelSettings.bAffectedBySteering)
		{
			CurrentWheelTurnRate = FMath::FInterpTo(CurrentWheelTurnRate, TargetWheelTurnRate, DeltaTime, 5.0f);
			float WheelSteeringDegree = CurrentWheelTurnRate * JetTyres[i].WheelSettings.MaxSteerAngle;
			WheelAnimVaribles[i].WheelRotation.Yaw = WheelSteeringDegree;

			WheelForwardDir = WheelForwardDir.RotateAngleAxis(WheelSteeringDegree, -SuspensionWorldAxis);
			WheelRightDir = WheelRightDir.RotateAngleAxis(WheelSteeringDegree, -SuspensionWorldAxis);
		}
		WheelForwardDir = FVector::VectorPlaneProject(WheelForwardDir, WheelsCacheValues[i].Normal);
		WheelForwardDir = WheelForwardDir.Normalize() ? WheelForwardDir : FVector::ZeroVector;
		WheelRightDir = FVector::VectorPlaneProject(WheelRightDir, WheelsCacheValues[i].Normal);
		WheelRightDir = WheelRightDir.Normalize() ? WheelRightDir : FVector::ZeroVector;

		FVector WheelRightVelocity = WheelPlaneVelocity.ProjectOnTo(WheelRightDir);
		FVector WheelForwardVelocity = WheelPlaneVelocity.ProjectOnTo(WheelForwardDir);

		if (FMath::Abs(WheelRightVelocity.Size()) > WheelStaticThreshold)
		{
			// Wheel slides horizontally
			FVector RightSuspensionForce = PlaneSuspensionForce.ProjectOnTo(WheelRightDir);
			float WheelFriction = FMath::Clamp(WheelRightVelocity.Size() * WheelSlideFrictionFactor, 0.0f, NormalSuspensionForce.Size() * FrictionRatio);
			WheelFrictionForce = -WheelRightVelocity / WheelRightVelocity.Size() * WheelFriction - RightSuspensionForce;
			//UE_LOG(LogTemp, Warning, TEXT("Wheel %d slides"), i);
		}
		else
		{
			FVector RightSuspensionForce = PlaneSuspensionForce.ProjectOnTo(WheelRightDir);
			float MaxStaticWheelFriction = NormalSuspensionForce.Size() * StaticFrictionRatio;
			if (RightSuspensionForce.Size() > MaxStaticWheelFriction)
			{
				// Wheel slides horizontally
				float SlideWheelFriction = NormalSuspensionForce.Size() * FrictionRatio;
				FVector TargetSlideWheelFrictionForce = -RightSuspensionForce / RightSuspensionForce.Size() * SlideWheelFriction;
				WheelFrictionForce = TargetSlideWheelFrictionForce;
				//UE_LOG(LogTemp, Warning, TEXT("Wheel %d about to slides"), i);
			}
			else
			{
				// wheel not slides
				WheelFrictionForce = -RightSuspensionForce;
				//UE_LOG(LogTemp, Warning, TEXT("Wheel %d not slides"), i);
			}
		}
		FVector ForwardSuspensionForce = PlaneSuspensionForce.ProjectOnTo(WheelForwardDir);
		float TargetWheelDrag = NormalSuspensionForce.Size() * JetTyres[i].WheelSettings.WheelDragRatio;
		if (JetTyres[i].WheelSettings.bAffectedByBrake && BrakeForceRatio > 0.0f)
		{
			float MaxStaticWheelFriction = NormalSuspensionForce.Size() * StaticFrictionRatio;
			float WheelBrake = MaxStaticWheelFriction * 0.5 * BrakeForceRatio;
			TargetWheelDrag += WheelBrake;
		}
		float WheelDrag = FMath::Clamp(WheelForwardVelocity.Size() * WheelSlideFrictionFactor, 0.0f, TargetWheelDrag);
		WheelDragForce = FMath::Abs(WheelForwardVelocity.Size()) > WheelStaticThreshold ? -WheelForwardVelocity / WheelForwardVelocity.Size() * WheelDrag : -ForwardSuspensionForce;

		WheelsForcesToAdd[i] += WheelFrictionForce + WheelDragForce;

		float WheelPitchDegree = WheelForwardVelocity.Size() * DeltaTime / JetTyres[i].WheelSettings.WheelRadius / 3.14f * 180.0f;
		float WheelPitchDir = WheelForwardVelocity.Dot(WheelForwardDir) > 0.0f ? 1.0f : -1.0f;
		WheelAnimVaribles[i].WheelRotation.Pitch -= WheelPitchDegree * WheelPitchDir;
		WheelAnimVaribles[i].WheelRotation.Pitch = WheelAnimVaribles[i].WheelRotation.Pitch > 360.0f ? WheelAnimVaribles[i].WheelRotation.Pitch - 360.0f : WheelAnimVaribles[i].WheelRotation.Pitch;
		WheelAnimVaribles[i].WheelRotation.Pitch = WheelAnimVaribles[i].WheelRotation.Pitch < 0.0f ? WheelAnimVaribles[i].WheelRotation.Pitch + 360.0f : WheelAnimVaribles[i].WheelRotation.Pitch;
		
		//DrawDebugLine(GetWorld(), WheelsCacheValues[i].ImpactLocation, WheelsCacheValues[i].ImpactLocation + WheelForwardDir * 1000.0f, FColor::Yellow, false, 0.0f);
	}
}

void UFlightPhysicsComponent::ThrusterForceTick(float DeltaTime)
{
	ThrustersCalculation(DeltaTime);
	if (JetMesh)
	{
		for (int i = 0; i < ThrusterForcesToAdd.Num(); ++i)
		{
			JetMesh->AddForceAtLocation(
				ThrusterForcesToAdd[i] * 100.0f,
				Fighter->GetTransform().TransformPosition(JetEngineSettings[i].EngineLocation)
			);
			/*DrawDebugDirectionalArrow(
				GetWorld(),
				Fighter->GetTransform().TransformPosition(JetEngineSettings[i].EngineLocation),
				Fighter->GetTransform().TransformPosition(JetEngineSettings[i].EngineLocation) + JetEngineSettings[i].EngineLocation + ThrusterForcesToAdd[i],
				7.0f,
				FColor::Yellow,
				false,
				0.0f
			);*/
		}
	}
}

void UFlightPhysicsComponent::ThrustersCalculation(float DeltaTime)
{
	for (int i = 0; i < JetEngineSettings.Num(); ++i)
	{
		if (ThrusterForcesToAdd.Num() < i + 1)
		{
			ThrusterForcesToAdd.Emplace();
			CurrentThrusters.Emplace();
		}

		float TargetThruster = CurrentThrusterRatio * JetEngineSettings[i].MaxNormalThrust * 9.8f;

		// Consider the effect of air density
		TargetThruster *= 1.0f;

		CurrentThrusters[i] = FMath::FInterpTo(CurrentThrusters[i], TargetThruster, DeltaTime, 3.0f);

		FVector ThrusterForcesAtLocal = JetEngineSettings[i].ThrustAixs * CurrentThrusters[i];

		ThrusterForcesToAdd[i] = Fighter->GetTransform().TransformVector(ThrusterForcesAtLocal);
	}
}

void UFlightPhysicsComponent::AerodynamicForceTick(float DeltaTime)
{
	AerodynamicForceCalculation(DeltaTime);
	AerodynamicAngularDampingCalculation(DeltaTime);

	for (int i = 0; i < MainWingsForcesToAdd.Num(); ++i)
	{
		JetMesh->AddForceAtLocation(
			MainWingsForcesToAdd[i] * 100.0f,
			JetMesh->GetComponentTransform().TransformPosition(AerodynamicSettings.MainWings[i].LiftForceLocation)
		);
	}
	for (int i = 0; i < StablizersForcesToAdd.Num(); ++i)
	{
		JetMesh->AddForceAtLocation(
			StablizersForcesToAdd[i] * 100.0f,
			JetMesh->GetComponentTransform().TransformPosition(AerodynamicSettings.Stablizers[i].LiftForceLocation)
		);
	}
	for (int i = 0; i < RuddersForcesToAdd.Num(); ++i)
	{
		JetMesh->AddForceAtLocation(
			RuddersForcesToAdd[i] * 100.0f,
			JetMesh->GetComponentTransform().TransformPosition(AerodynamicSettings.Rudders[i].LiftForceLocation)
		);
	}
	if (AerodynamicSettings.CentralLiftBody.bCanWork)
	{
		JetMesh->AddForce(CentralLiftForceToAdd * 100.0f);
	}

	JetMesh->AddTorqueInRadians(AngularDampingTorque * 10000.0f);
}

void UFlightPhysicsComponent::AerodynamicForceCalculation(float DeltaTime)
{
	/** Calculate MainWings */
	for (int i = 0; i < AerodynamicSettings.MainWings.Num(); ++i)
	{
		if (MainWingsForcesToAdd.Num() < i + 1)
		{
			MainWingsForcesToAdd.Emplace();
			AileronsPitch.Emplace();
			FlapsPitch.Emplace();
		}

		FQuat CurrentQuat = FQuat(AerodynamicSettings.MainWings[i].MainWingRotation);

		float AdditionalPitch = 0.0f;
		if (AerodynamicSettings.MainWings[i].Aileron.bCanWork)
		{
			float AileronPitch = FMath::GetMappedRangeValueClamped(FVector2D(-1.0f, 1.0f), AerodynamicSettings.MainWings[i].Aileron.RotateRange, CurrentControlX);
			AileronPitch = i == 1 ? -AileronPitch : AileronPitch;
			AileronsPitch[i] = AileronPitch;
			AdditionalPitch += AileronPitch * 0.2f;
		}
		if (AerodynamicSettings.MainWings[i].Flap.bCanWork)
		{
			float FlapPitch = FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 1.0f), AerodynamicSettings.MainWings[i].Flap.RotateRange, CurrentControlFlaps);
			FlapsPitch[i] = FlapPitch;
			AdditionalPitch += FlapPitch * 0.3f;
		}
		FQuat AddtionalQuat = FQuat(FRotator(AdditionalPitch, 0.0f, 0.0f));
		CurrentQuat = CurrentQuat * AddtionalQuat;

		FRotator CurrentRotation = CurrentQuat.Rotator();
		MainWingsForcesToAdd[i] = WingForceCalculate(
			AerodynamicSettings.MainWings[i].LiftForceLocation,
			CurrentRotation,
			AerodynamicSettings.MainWings[i].CoefficientOfLiftCurve,
			AerodynamicSettings.MainWings[i].CoefficientOfDragCurve,
			AerodynamicSettings.MainWings[i].WingArea
		);
	}

	/** Calculate Stablizers */
	for (int i = 0; i < AerodynamicSettings.Stablizers.Num(); ++i)
	{
		if (StablizersForcesToAdd.Num() < i + 1)
		{
			StablizersForcesToAdd.Emplace();
			StablizersPitch.Emplace();
		}
		float StablizerPitch = FMath::GetMappedRangeValueClamped(FVector2D(-1.0f, 1.0f), AerodynamicSettings.Stablizers[i].RotateRange, CurrentControlY);
		StablizersPitch[i] = StablizerPitch;
		FQuat AddtionalQuat = FQuat(FRotator(StablizerPitch, 0.0f, 0.0f));
		FQuat InitialQuat = FQuat(AerodynamicSettings.Stablizers[i].InitialRotation);
		FQuat CurrentQuat = InitialQuat * AddtionalQuat;
		FRotator StablizerRotation = CurrentQuat.Rotator();
		StablizersForcesToAdd[i] = WingForceCalculate(
			AerodynamicSettings.Stablizers[i].LiftForceLocation,
			StablizerRotation,
			AerodynamicSettings.Stablizers[i].CoefficientOfLiftCurve,
			AerodynamicSettings.Stablizers[i].CoefficientOfDragCurve,
			AerodynamicSettings.Stablizers[i].WingArea
		);
	}

	/** Calculate Rudders */
	for (int i = 0; i < AerodynamicSettings.Rudders.Num(); ++i)
	{
		if (RuddersForcesToAdd.Num() < i + 1)
		{
			RuddersForcesToAdd.Emplace();
			RuddersYaw.Emplace();
		}
		float RudderYaw = FMath::GetMappedRangeValueClamped(FVector2D(-1.0f, 1.0f), AerodynamicSettings.Rudders[i].RotateRange, CurrentControlRudders);
		RuddersYaw[i] = -RudderYaw;

		FQuat CurrentQuat = FQuat(AerodynamicSettings.Rudders[i].InitialRotation);

		RudderYaw = i == 1 ? -RudderYaw : RudderYaw;
		FQuat AddtionalQuat = FQuat(FRotator(RudderYaw * 0.3f, 0.0f, 0.0f));
		CurrentQuat = CurrentQuat * AddtionalQuat;
		FRotator CurrentRotation = CurrentQuat.Rotator();
		RuddersForcesToAdd[i] = WingForceCalculate(
			AerodynamicSettings.Rudders[i].LiftForceLocation,
			CurrentRotation,
			AerodynamicSettings.Rudders[i].CoefficientOfLiftCurve,
			AerodynamicSettings.Rudders[i].CoefficientOfDragCurve,
			AerodynamicSettings.Rudders[i].WingArea
		);
	}

	/** Calculate Central Body Lift */
	if (AerodynamicSettings.CentralLiftBody.bCanWork)
	{
		CentralLiftForceToAdd = WingForceCalculate(
			FVector::ZeroVector,
			AerodynamicSettings.CentralLiftBody.InitialRotation,
			AerodynamicSettings.CentralLiftBody.CoefficientOfLiftCurve,
			AerodynamicSettings.CentralLiftBody.CoefficientOfDragCurve,
			AerodynamicSettings.CentralLiftBody.LiftBodyArea,
			true,
			AerodynamicSettings.CentralLiftBody.SideBodyArea
		);
	}
}

void UFlightPhysicsComponent::AerodynamicAngularDampingCalculation(float DeltaTime)
{
	float JetAngularSpeedPitch = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetRightVector());
	float JetAngularSpeedRoll = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetForwardVector());
	float JetAngularSpeedYaw = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetUpVector());

	FVector PitchDampingTorque = -AerodynamicSettings.PitchDampingCoefficient * JetAngularSpeedPitch * JetMesh->GetRightVector() * 1000.0f;
	FVector RollDampingTorque = -AerodynamicSettings.RollDampingCoefficient * JetAngularSpeedRoll * JetMesh->GetForwardVector() * 1000.0f;
	FVector YawDampingTorque = -AerodynamicSettings.YawDampingCoefficient * JetAngularSpeedYaw * JetMesh->GetUpVector() * 1000.0f;

	AngularDampingTorque = PitchDampingTorque + RollDampingTorque + YawDampingTorque;

	//DrawDebugLine(GetWorld(), JetMesh->GetComponentLocation(), JetMesh->GetComponentLocation() + RollDampingTorque * 10.0f, FColor::Yellow, false, 0.0f);
}

FVector UFlightPhysicsComponent::WingForceCalculate(const FVector& WingLoc, const FRotator& WingRot, UCurveFloat* ClCurve, UCurveFloat* CdCurve, const float& Area, bool bSideDragExist, const float& SideArea)
{
	FVector WingLocation = JetMesh->GetComponentTransform().TransformPosition(WingLoc);

	FQuat WingWorldQuat = JetMesh->GetComponentTransform().TransformRotation(FQuat(WingRot));
	FVector WingForwardDir = WingWorldQuat.RotateVector(FVector::ForwardVector);
	FVector WingUpDir = WingWorldQuat.RotateVector(FVector::UpVector);
	FVector WingRight = WingWorldQuat.RotateVector(FVector::RightVector);

	FVector WingPlaneVelocity = FVector::VectorPlaneProject(JetMeshVelocity, WingRight);
	FVector WingRightVelocity = JetMeshVelocity - WingPlaneVelocity;

	/** Calculate Angle of Attack */
	FVector WingPlaneVelocityDir = WingPlaneVelocity;
	WingPlaneVelocityDir.Normalize();

	float SignValue = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(WingPlaneVelocityDir, WingUpDir))) < 90.0f ? -1.0f : 1.0f;
	float AoA = SignValue * FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(WingPlaneVelocityDir, WingForwardDir)));

	float Speed = WingPlaneVelocity.Size() / 100.0f;
	float RightSpeed = WingRightVelocity.Size() / 100.0f;

	float AirDensity = AirEnvironmentManager? AirEnvironmentManager->GetAirDensityAtLocation(WingLocation) : 1.225f;

	float Cl = ClCurve ? ClCurve->GetFloatValue(AoA) : 0.0f;
	float Cd = CdCurve ? CdCurve->GetFloatValue(AoA) : 0.0f;
	float Cf = Speed > 0.001f ? RightSpeed / Speed : 0.0f;

	float Lift = 0.5f * Cl * AirDensity * Speed * Speed * Area;
	float Drag = 0.5f * Cd * AirDensity * Speed * Speed * Area;
	float SideFriction = 0.5f * Cf * AirDensity * RightSpeed * RightSpeed * Area;
	float SideDrag = bSideDragExist ? 0.5f * 330.0f * AirDensity * RightSpeed * RightSpeed * SideArea : 0.0f;

	FQuat RotateWingUp = FQuat(WingRight, FMath::DegreesToRadians(AoA));
	FVector LiftForceDir = RotateWingUp.RotateVector(WingUpDir);
	FVector FrictionForceDir = -WingRightVelocity;
	FrictionForceDir.Normalize();

	FVector LiftForce = LiftForceDir * Lift;
	FVector DragForce = -WingPlaneVelocityDir * Drag;
	FVector SideFrictionForce = FrictionForceDir * SideFriction;
	FVector SideDragForce = FrictionForceDir * SideDrag;

	FVector WingForce = LiftForce + DragForce + SideFrictionForce + SideDragForce;

	/*DrawDebugLine(GetWorld(), DrawLocation, DrawLocation + WingPlaneVelocity * 80.0f, FColor::Red, false, 0.0f);
	DrawDebugLine(GetWorld(), DrawLocation, DrawLocation + LiftForce * 80.0f, FColor::Green, false, 0.0f);
	DrawDebugLine(GetWorld(), DrawLocation, DrawLocation + DragForce * 80.0f, FColor::Yellow, false, 0.0f);*/
	//DrawDebugLine(GetWorld(), DrawLocation, DrawLocation + WingForwardDir * 1000.0f, FColor::Green, false, 0.0f);
	//DrawDebugLine(GetWorld(), DrawLocation, DrawLocation + WingUpDir * 1000.0f, FColor::Yellow, false, 0.0f);
	//AddDebugMessageOnScreen(0.0f, FColor::Cyan, FString::Printf(TEXT("AirDensity: %f"), AirDensity));

	return WingForce;
}

void UFlightPhysicsComponent::FlyingControlTick(float DeltaTime)
{
	if (1)
	{
		float JetAngularSpeedRoll = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetForwardVector());
		AddDebugMessageOnScreen(0.0f, FColor::Red, FString::Printf(TEXT("JetAngularSpeedRoll: %f"), JetAngularSpeedRoll));

		float JetAngularSpeedPitch = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetRightVector());
		AddDebugMessageOnScreen(0.0f, FColor::Red, FString::Printf(TEXT("JetAngularSpeedPitch: %f"), JetAngularSpeedPitch));
	}

	if (!bFlyingControlWork)
	{
		CurrentControlX = FMath::FInterpTo(CurrentControlX, ControlStickX, DeltaTime, 5.0f);
		CurrentControlY = FMath::FInterpTo(CurrentControlY, ControlStickY, DeltaTime, 5.0f);
		CurrentControlRudders = FMath::FInterpTo(CurrentControlRudders, ControlRudders, DeltaTime, 5.0f);
		CurrentControlFlaps = FMath::FInterpTo(CurrentControlFlaps, ControlFlaps, DeltaTime, 5.0f);
		return;
	}

	if (FMath::Abs(ControlStickX) < 0.01f)
	{
		float JetAngularSpeedRoll = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetForwardVector());
		
		TargetStickX = FMath::GetMappedRangeValueClamped(FVector2D(-5.0f, 5.0f), FVector2D(-1.0f, 1.0f), JetAngularSpeedRoll);
	}
	else
	{
		TargetStickX = ControlStickX;
	}

	if (FMath::Abs(ControlStickY) < 0.01f)
	{
		float JetAngularSpeedPitch = JetMeshAngularVelocityInRadians.Dot(JetMesh->GetRightVector());
		TargetStickY = FMath::GetMappedRangeValueClamped(FVector2D(-0.3f, 0.3f), FVector2D(-1.0f, 1.0f), JetAngularSpeedPitch); 
		TargetStickY = -TargetStickY;
	}
	else
	{	
		if (ControlStickY > 0.0f)
		{
			if (GForce > FlyingControlSystemParameters.GForceLimit.X + 0.1f)
			{
				SuitableControlYUpLimit = FMath::FInterpTo(SuitableControlYUpLimit, ControlStickY, DeltaTime, 5.0f);
			}
			else if (GForce < FlyingControlSystemParameters.GForceLimit.X - 0.1f)
			{
				float OverloadRatio = GForce / FlyingControlSystemParameters.GForceLimit.X;
				float controlInputAdjustment = FMath::Clamp(1.0f / OverloadRatio, 0.0f, 1.0f);
				SuitableControlYUpLimit = FMath::FInterpTo(SuitableControlYUpLimit, SuitableControlYUpLimit * controlInputAdjustment, DeltaTime, 2.0f);
			}
			TargetStickY = FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 1.0f), FVector2D(0.0f, SuitableControlYUpLimit), ControlStickY);
		}
		else
		{
			if (GForce < FlyingControlSystemParameters.GForceLimit.Y - 0.1f)
			{
				SuitableControlYDownLimit = FMath::FInterpTo(SuitableControlYDownLimit, ControlStickY, DeltaTime, 5.0f);
			}
			else if (GForce > FlyingControlSystemParameters.GForceLimit.Y + 0.1f)
			{
				float OverloadRatio = GForce / FlyingControlSystemParameters.GForceLimit.Y;
				float controlInputAdjustment = FMath::Clamp(1.0f / OverloadRatio, 0.0f, 1.0f);
				SuitableControlYDownLimit = FMath::FInterpTo(SuitableControlYDownLimit, SuitableControlYDownLimit * controlInputAdjustment, DeltaTime, 2.0f);
			}
			TargetStickY = FMath::GetMappedRangeValueClamped(FVector2D(-1.0f, 0.0f), FVector2D(SuitableControlYDownLimit, 0.0f), ControlStickY);

			AddDebugMessageOnScreen(0.0f, FColor::Blue, FString::Printf(TEXT("SuitableControlYDownLimit: %f"), SuitableControlYDownLimit));
		}

	}

	CurrentControlX = FMath::FInterpTo(CurrentControlX, TargetStickX, DeltaTime, 5.0f);
	CurrentControlY = FMath::FInterpTo(CurrentControlY, TargetStickY, DeltaTime, 5.0f);
	CurrentControlRudders = FMath::FInterpTo(CurrentControlRudders, ControlRudders, DeltaTime, 5.0f);
	CurrentControlFlaps = FMath::FInterpTo(CurrentControlFlaps, ControlFlaps, DeltaTime, 5.0f);
}

float UFlightPhysicsComponent::CalculateSuitableControlValueForTick(const float& CurrentControlValue, const float& TargetControlValue, const float& CurrentConditionAmount, const float& LimitConditionAmount, float DeltaTime)
{
	if (LimitConditionAmount == 0)
	{
		return 0.0f;
	}

	float SuitableControlValue;
	
	if (FMath::Abs(CurrentConditionAmount) <= FMath::Abs(LimitConditionAmount))
	{
		SuitableControlValue = FMath::FInterpTo(CurrentControlValue, TargetControlValue, DeltaTime, 1.0f);
	}
	else
	{
		float OverloadRatio = CurrentConditionAmount / LimitConditionAmount - 1.0f;
		float adjustmentFactor = 1.0f;
		float controlInputAdjustment = FMath::Exp(-OverloadRatio * adjustmentFactor);
		controlInputAdjustment = FMath::Clamp(controlInputAdjustment, 0.0f, 1.0f);
		SuitableControlValue = CurrentControlValue * controlInputAdjustment;
	}
	return SuitableControlValue;
}

void UFlightPhysicsComponent::SetWheelsBrake(float AxisValue)
{
	BrakeForceRatio = FMath::Clamp(AxisValue, 0.0f, 1.0f);
}

void UFlightPhysicsComponent::SetSteeringWheels(float AxisValue)
{
	TargetWheelTurnRate = FMath::Clamp(AxisValue, -1.0f, 1.0f);
}

void UFlightPhysicsComponent::SetAddThruster(float AxisValue)
{
	float Axis = FMath::Clamp(AxisValue, -1.0f, 1.0f);

	float TargetCurrentThrusterRatio = CurrentThrusterRatio + Axis * ThrusterRatioAddPerSecond * GetWorld()->GetDeltaSeconds();

	CurrentThrusterRatio = FMath::Clamp(TargetCurrentThrusterRatio, 0.0f, 1.0f);
}

void UFlightPhysicsComponent::SetControlStickXAxis(float AxisValue)
{
	ControlStickX = FMath::Clamp(AxisValue, -1.0f, 1.0f);
}

void UFlightPhysicsComponent::SetControlStickYAxis(float AxisValue)
{
	ControlStickY = FMath::Clamp(AxisValue, -1.0f, 1.0f);
}

void UFlightPhysicsComponent::SetRudders(float AxisValue)
{
	ControlRudders = FMath::Clamp(AxisValue, -1.0f, 1.0f);
}

void UFlightPhysicsComponent::SetFlaps(float AxisValue)
{
	ControlFlaps= FMath::Clamp(AxisValue, -1.0f, 1.0f);
}

void UFlightPhysicsComponent::SetFlyingControlSystemActivated(bool bIsActivated)
{
	bFlyingControlWork = bIsActivated;
}

void UFlightPhysicsComponent::InitializeAnimationInstance()
{
	if (!JetMesh || !JetMesh->GetAnimInstance()) return;

	JetAnimInstance = JetAnimInstance == nullptr ? Cast<UFighterJetAnimInstance>(JetMesh->GetAnimInstance()) : JetAnimInstance;
	if (JetAnimInstance)
	{
		if (JetAnimInstance->GetFlightComponent() == nullptr)
		{
			JetAnimInstance->SetFlightComponent(this);
		}
	}
}
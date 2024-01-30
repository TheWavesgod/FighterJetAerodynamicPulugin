// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FlightPhysicsComponent.generated.h"

USTRUCT(BlueprintType)
struct FWheelSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	float WheelRadius;

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	float WheelWidth;

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	bool bAffectedBySteering;

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	float MaxSteerAngle;

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	bool bAffectedByBrake;

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	float WheelFrictionRatio = 0.7;

	UPROPERTY(EditAnywhere, Category = "Wheel Settings")
	float WheelDragRatio = 0.05;
};

USTRUCT(BlueprintType)
struct FChassisSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Chassis Settings")
	FVector SuspensionAxis = FVector(0.0f, 0.0f, -1.0f);

	UPROPERTY(EditAnywhere, Category = "Chassis Settings")
	float SuspensionMaxRaise = 10.0f;

	UPROPERTY(EditAnywhere, Category = "Chassis Settings")
	float SuspensionMaxDrop = 10.0f;

	UPROPERTY(EditAnywhere, Category = "Chassis Settings")
	float SuspensionDampingRatio = 0.5;
	
	UPROPERTY(EditAnywhere, Category = "Chassis Settings")
	float SpringRate = 250.0f;

	/** The Chassis preload length > 0 */
	UPROPERTY(EditAnywhere, Category = "Chassis Settings")
	float SpringPreLoadLength;
};

USTRUCT(BlueprintType)
struct FJetTyreclass
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	FName WheelBoneName;

	UPROPERTY(EditAnywhere)
	FVector WheelLocationOffset;

	UPROPERTY(EditAnywhere)
	FWheelSettings WheelSettings;

	UPROPERTY(EditAnywhere)
	FChassisSettings ChassisSettings;
};

/**
 * Dynamic performance parameters of aircraft
 */
USTRUCT(BlueprintType)
struct FJetEngineSettings
{
	GENERATED_BODY()

	/** Relative location */
	UPROPERTY(EditAnywhere, Category = "Jet Engine Settings")
	FVector EngineLocation;

	/** Maximum normal engine thrust Unit Kg */
	UPROPERTY(EditAnywhere, Category = "Jet Engine Settings")
	float MaxNormalThrust;

	/** Maximum engine thrust Unit Kg */
	UPROPERTY(EditAnywhere, Category = "Jet Engine Settings")
	float MaxExtraThrust;

	UPROPERTY(EditAnywhere, Category = "Jet Engine Settings")
	FVector ThrustAixs = FVector(-1.0f, 0.0f, 0.0f);
};

USTRUCT(BlueprintType)
struct FControlFace
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	bool bCanWork;

	UPROPERTY(EditAnywhere)
	FVector2D RotateRange = FVector2D(-30.0f, 30.0f);
};

USTRUCT(BlueprintType)
struct FJetMainWings
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	FVector LiftForceLocation;

	UPROPERTY(EditAnywhere)
	FRotator MainWingRotation;

	UPROPERTY(EditAnywhere)
	float WingArea;

	UPROPERTY(EditAnywhere)
	FControlFace Aileron;

	UPROPERTY(EditAnywhere)
	FControlFace Flap;

	UPROPERTY(EditAnywhere)
    UCurveFloat* CoefficientOfLiftCurve;

	UPROPERTY(EditAnywhere)
    UCurveFloat* CoefficientOfDragCurve;
};

USTRUCT(BlueprintType)
struct FJetAuxiliaryWing
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	FVector LiftForceLocation;

	UPROPERTY(EditAnywhere)
	FRotator InitialRotation;

	UPROPERTY(EditAnywhere)
	FVector2D RotateRange = FVector2D(-30.0f, 30.0f);

	UPROPERTY(EditAnywhere)
	float WingArea;

	UPROPERTY(EditAnywhere)
    UCurveFloat* CoefficientOfLiftCurve;

	UPROPERTY(EditAnywhere)
    UCurveFloat* CoefficientOfDragCurve;
};

USTRUCT(BlueprintType)
struct FCentralLiftBody
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	bool bCanWork = false;

	UPROPERTY(EditAnywhere)
	FRotator InitialRotation;

	UPROPERTY(EditAnywhere)
	float LiftBodyArea;

	UPROPERTY(EditAnywhere)
	float SideBodyArea;

	UPROPERTY(EditAnywhere)
    UCurveFloat* CoefficientOfLiftCurve;

	UPROPERTY(EditAnywhere)
    UCurveFloat* CoefficientOfDragCurve;
};

/**
 * Jet aerodynamic parameters settings
 */
USTRUCT(BlueprintType)
struct FAeroDynamicSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	TArray<FJetMainWings> MainWings;

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	TArray<FJetAuxiliaryWing> Stablizers;

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	TArray<FJetAuxiliaryWing> Rudders;

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	FCentralLiftBody CentralLiftBody;

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	float PitchDampingCoefficient = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	float RollDampingCoefficient = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Jet AeroDynamic Settings")
	float YawDampingCoefficient = 0.0f;
};

USTRUCT(BlueprintType)
struct FFlyingControlSystemParameters
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "FlyingControlSystemParameters")
	FVector2D GForceLimit = FVector2D(-3.0f, 9.0f);

	UPROPERTY(EditAnywhere, Category = "FlyingControlSystemParameters")
	float MaxRollSpeedInRadians = 5.0f;

};


USTRUCT()
struct FWheelCacheVaribles
{
	GENERATED_BODY()

	bool bIsHit;
	FVector LastFrameImpactLocation;
	FVector ImpactLocation;
	float LastFrameDistance;
	float Distance;
	FVector Normal;
	UPhysicalMaterial* SurfaceMatrial;
};

USTRUCT()
struct FWheelAnimVaribles
{
	GENERATED_BODY()

	float SuspensionDisplacement;
	FRotator WheelRotation;
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class FLIGHTJET_API UFlightPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UFlightPhysicsComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	friend class AFighter;

	/** Axis Value between 0 - 1 */
	UFUNCTION(BlueprintCallable)
	void SetWheelsBrake(float AxisValue);

	/** Axis Value between -1 - 1 */
	UFUNCTION(BlueprintCallable)
	void SetSteeringWheels(float AxisValue);

	/** Axis Value between -1 - 1 will add to throttle*/
	UFUNCTION(BlueprintCallable)
	void SetAddThruster(float AxisValue);

	UFUNCTION(BlueprintCallable)
	void SetControlStickXAxis(float AxisValue);

	UFUNCTION(BlueprintCallable)
	void SetControlStickYAxis(float AxisValue);

	UFUNCTION(BlueprintCallable)
	void SetRudders(float AxisValue);

	UFUNCTION(BlueprintCallable)
	void SetFlaps(float AxisValue);

	UFUNCTION(BlueprintCallable)
	void SetFlyingControlSystemActivated(bool bIsActivated);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	void InitializeAnimationInstance();

	// Debug
	void DebugEveryTick();
	void AddDebugMessageOnScreen(const float DisplayTime, const FColor Color, const FString DiplayString);

private:
	class AFighter* Fighter;
	class USkeletalMeshComponent* JetMesh;
	class UFighterJetAnimInstance* JetAnimInstance;

	UPROPERTY()
	class AAirEnvironmentManager* AirEnvironmentManager;

	/**
	 *  Jet Fighter's Parameters need to be Modified
	 */
	 // Jet's empty weight, the unit is kg
	 UPROPERTY(EditAnywhere, Category = "Custom Parameters")
	 float JetEmptyWeight;

	 // The center of mass of aircraft's body, base in local location
	 UPROPERTY(EditAnywhere, Category = "Custom Parameters")
	 FVector CenterOfMass;

	 // Jet Engine Settings
	 UPROPERTY(EditAnywhere, Category = "Custom Parameters")
	 TArray<FJetEngineSettings> JetEngineSettings;

	 // Tyres Settings
	 UPROPERTY(EditAnywhere, Category = "Custom Parameters")
	 TArray<FJetTyreclass> JetTyres;

	 // Aerodynamic Settings
	 UPROPERTY(EditAnywhere, Category = "Custom Parameters")
     FAeroDynamicSettings AerodynamicSettings;

	 // Flying Control System Settings
	 UPROPERTY(EditAnywhere, Category = "Custom Parameters")
	 FFlyingControlSystemParameters FlyingControlSystemParameters;

	 /**
	  * Function Use for Wheels Calculating
	  */
	  void WheelsForceTick(float DeltaTime);

	  void WheelsRaycastAndCache();

	  void WheelsCalculateForces(float DeltaTime);
	  // extra length used for wheel Raycast
	  float WheelRayOffset = 100.0f;
	  UPROPERTY(EditAnywhere, Category = "WheelFrictionDebug Parameters")
	  float WheelStaticThreshold = 0.05; 

	  UPROPERTY(EditAnywhere, Category = "WheelFrictionDebug Parameters")
	  float WheelSlideFrictionFactor = 30.0f;

	  TArray<FWheelCacheVaribles> WheelsCacheValues;
	  TArray<FVector> WheelsForcesToAdd;

	  /** Wheels States valuables */
	  float BrakeForceRatio;
	  FVector WheelFrictionForce;
	  FVector WheelDragForce;
	  float TargetWheelTurnRate = 0.0f;
	  float CurrentWheelTurnRate = 0.0f;

	  /**
		* Function use for Thruster calculation
		*/	
		void ThrusterForceTick(float DeltaTime);

		void ThrustersCalculation(float DeltaTime);

	  /** Thruster state values */
	  float CurrentThrusterRatio;
	  UPROPERTY(EditAnywhere, Category = "WheelFrictionDebug Parameters")
	  float ThrusterRatioAddPerSecond = 0.2f; 

	  TArray<float> CurrentThrusters;

	  TArray<FVector> ThrusterForcesToAdd;

	  /**
	   * 
	   */
	   void JetParametersTick(float DeltaTime);
	   float CalculateCurrentGForce(float DeltaTime);

	   /** Jet's state values */
	   FVector JetMeshVelocity;
	   FVector LastFrameJetMeshVelocity;
	   FVector JetMeshAcceleration;

	   FVector JetMeshAngularVelocityInRadians;

	   float GroundSpeed = 0.0f;
	   float AngleOfAttack = 0.0f;

	   float GForce;

	   /**
	    * Function use for aerodynamic calculation
	    */
		void AerodynamicForceTick(float DeltaTime);
		void AerodynamicForceCalculation(float DeltaTime);
		void AerodynamicAngularDampingCalculation(float DeltaTime);

		FVector WingForceCalculate(const FVector& WingLoc, const FRotator& WingRot, UCurveFloat* ClCurve, UCurveFloat* CdCurve, const float& Area, bool bSideDragExist = false, const float& SideArea = 0.0f);

		/** Aerodynamic State Values */
		float ControlStickX = 0.0f;
		float ControlStickY = 0.0f;
		float ControlRudders = 0.0f;
		float ControlFlaps = 0.0f;

		float CurrentControlX = 0.0f;
		float CurrentControlY = 0.0f;
		float CurrentControlRudders = 0.0f;
		float CurrentControlFlaps = 0.0f;

		TArray<FVector> MainWingsForcesToAdd;
		TArray<FVector> StablizersForcesToAdd;
		TArray<FVector> RuddersForcesToAdd;
		FVector CentralLiftForceToAdd;
		FVector AngularDampingTorque = FVector::ZeroVector;

		/**
		 * Function use for calculating flying control 
		 */
		 void FlyingControlTick(float DeltaTime);

		 float CalculateSuitableControlValueForTick(const float& CurrentControlValue, const float& TargetControlValue, const float& CurrentConditionAmount, const float& LimitConditionAmount, float DeltaTime);

		/** Values used for  calculating flying control */
		bool bFlyingControlWork = true;

		float TargetStickX = 0.0f;
		float TargetStickY = 0.0f;

		float SuitableControlYUpLimit;
		float SuitableControlYDownLimit;

public:
	TArray<FWheelAnimVaribles> WheelAnimVaribles;
	TArray<float> StablizersPitch;
	TArray<float> AileronsPitch;
	TArray<float> FlapsPitch;
	TArray<float> RuddersYaw;

	UFUNCTION(BlueprintPure)
	FORCEINLINE float GetCurrentThrusterRatio() const { return CurrentThrusterRatio; }

	UFUNCTION(BlueprintPure)
	FORCEINLINE float GetStablizerPitch(int32 Index) const { return StablizersPitch[Index]; }

	UFUNCTION(BlueprintPure)
	FORCEINLINE float GetGroundSpeed() const { return GroundSpeed; }

	UFUNCTION(BlueprintPure)
	FORCEINLINE float GetCurrentAoA() const { return AngleOfAttack; }

	UFUNCTION(BlueprintCallable)
	FORCEINLINE void SetAirEnvironmentManager(AAirEnvironmentManager* Value) { AirEnvironmentManager = Value; }

	UFUNCTION(BlueprintPure)
	FORCEINLINE float GetCurrentGForce() const { return GForce; }

	UFUNCTION(BlueprintPure)
	FORCEINLINE bool GetFlyingControlSystemState() const { return bFlyingControlWork; }
};

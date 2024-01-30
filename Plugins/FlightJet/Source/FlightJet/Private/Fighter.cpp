#include "Fighter.h"
#include "Components/SkeletalMeshComponent.h"

AFighter::AFighter()
{
	PrimaryActorTick.bCanEverTick = false;

	JetMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("JetMesh"));
	SetRootComponent(JetMesh);

	FlightPhysicsComponent = CreateDefaultSubobject<UFlightPhysicsComponent>(TEXT("FlightPhysicsComponent"));
}

void AFighter::BeginPlay()
{
	Super::BeginPlay();
	
}

void AFighter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AFighter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}


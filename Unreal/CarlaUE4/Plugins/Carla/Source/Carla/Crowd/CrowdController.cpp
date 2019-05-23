#include "CrowdController.h"

ACrowdController::ACrowdController(void)
{
  PrimaryActorTick.bCanEverTick = false; 
}

ACrowdController::ACrowdController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = false;
}

void ACrowdController::PostInitializeComponents()
{
}

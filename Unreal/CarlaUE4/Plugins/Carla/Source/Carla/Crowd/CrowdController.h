#pragma once

#include "CrowdController.generated.h"

UCLASS()
class CARLA_API ACrowdController : public AActor
{
  GENERATED_BODY()

public:

  ACrowdController();

  ACrowdController(const FObjectInitializer &InObjectInitializer);
  
  void PostInitializeComponents() override;

  void SetEpisoide(UCarlaEpisode *ThisEpisode)
  {
    Episode = ThisEpisode;
  }

private:

  UCarlaEpisode *Episode = nullptr; 
    
};

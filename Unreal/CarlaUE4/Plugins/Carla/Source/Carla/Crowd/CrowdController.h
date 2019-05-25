#pragma once

#include "Map/RoadMap.h"
#include "Carla.h"
#include "CrowdController.generated.h"

UCLASS()
class CARLA_API ACrowdController : public AActor {
  GENERATED_BODY()

public:

  ACrowdController();

  ACrowdController(const FObjectInitializer &InObjectInitializer);

  void SetEpisode(UCarlaEpisode *Ep) {
    Episode = Ep;
  }

  void Initialize();
  
  void Tick(float DeltaSeconds) final;

private:

  UCarlaEpisode* Episode = nullptr;
  FRoadMap RoadMap;
  
  const FActorDefinition& RandWalkerActorDefinition() const;

};

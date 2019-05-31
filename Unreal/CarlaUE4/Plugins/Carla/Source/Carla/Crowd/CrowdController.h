#pragma once

#include "Map/RoadMap.h"
#include "Carla.h"
#include <carla/opendrive/OpenDriveParser.h>
#include <carla/road/Map.h>

#include "CrowdController.generated.h"

UCLASS()
class CARLA_API ACrowdController : public AActor {
  GENERATED_BODY()

public:

  ACrowdController();

  ACrowdController(const FObjectInitializer &InObjectInitializer);

  void SetEpisode(UCarlaEpisode *Ep) { Episode = Ep; }

  void SetRoadMap(const FRoadMap* RdMap) { RoadMap = RdMap; }

  void SetWaypointMap(const carla::road::Map* WpMap) { WaypointMap = WpMap; }

  void Initialize();
  
  void Tick(float DeltaSeconds) final;

private:

  UCarlaEpisode* Episode = nullptr;
  const FRoadMap* RoadMap;
  const carla::road::Map* WaypointMap;
  
  const FActorDefinition& RandWalkerActorDefinition() const;

};

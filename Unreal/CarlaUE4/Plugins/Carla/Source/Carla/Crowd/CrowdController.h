#pragma once

#include "Map/RoadMap.h"
#include "Carla.h"
#include <carla/opendrive/OpenDriveParser.h>
#include <carla/road/Map.h>
#include "CrowdWalker.h"

#include "CrowdController.generated.h"

UCLASS()
class CARLA_API ACrowdController : public AActor {
  GENERATED_BODY()

public:

  ACrowdController();

  ACrowdController(const FObjectInitializer &InObjectInitializer);

  void SetEpisode(UCarlaEpisode *InEpisode) { Episode = InEpisode; }

  void SetRoadMap(const FRoadMap* InRoadMap) { RoadMap = InRoadMap; }

  void SetWaypointMap(carla::road::Map* InWaypointMap) { WaypointMap = InWaypointMap; }
  
  void Tick(float DeltaSeconds) final;

private:

  UCarlaEpisode* Episode = nullptr;
  const FRoadMap* RoadMap;
  carla::road::Map* WaypointMap;
  TArray<CrowdWalker> Walkers;
  
  const FActorDefinition& RandWalkerActorDefinition() const;

};

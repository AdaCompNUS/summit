#pragma once

#include "OccupancyMap/OccupancyMap.h"
#include "Carla.h"
#include <carla/opendrive/OpenDriveParser.h>
#include <carla/road/Map.h>
#include "CrowdWalker.h"
#include "RVO//RVO.h"

#include "CrowdController.generated.h"

UCLASS()
class CARLA_API ACrowdController : public AActor {
  GENERATED_BODY()

public:

  ACrowdController();

  ACrowdController(const FObjectInitializer &InObjectInitializer);

  void SetEpisode(UCarlaEpisode *InEpisode) { Episode = InEpisode; }

  void SetOccupancyMap(const FOccupancyMap* InOccupancyMap) { OccupancyMap = InOccupancyMap; }

  void SetWaypointMap(carla::road::Map* InWaypointMap) { WaypointMap = InWaypointMap; }

  void SetBounds(const FBox2D& InBounds) { Bounds = InBounds; }

  void Tick(float DeltaSeconds) final;

  void StartCrowd(int NumWalkers);

  void StopCrowd();

private:

  UCarlaEpisode* Episode = nullptr;
  
  const FOccupancyMap* OccupancyMap;
  carla::road::Map* WaypointMap;
  FBox2D Bounds;

  FOccupancyArea OccupancyArea;

  TArray<CrowdWalker> Walkers;
  RVO::RVOSimulator RVOSim;
  
  const FActorDefinition& RandWalkerActorDefinition() const;

};

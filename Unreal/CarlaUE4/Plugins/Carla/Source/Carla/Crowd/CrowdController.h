#pragma once

#include "Polygon.h"
#include "CrowdController.generated.h"

UCLASS()
class CARLA_API ACrowdController : public AActor {
  GENERATED_BODY()

public:

  ACrowdController();

  ACrowdController(const FObjectInitializer &InObjectInitializer);
  
  void PostInitializeComponents() override;

  void SetEpisoide(UCarlaEpisode *ThisEpisode) {
    Episode = ThisEpisode;
  }

  FVector2D RandomRoadPoint() const;

private:

  UCarlaEpisode* Episode = nullptr; 
  TArray<FPolygon> RoadPolygons;
  float TotalRoadArea = 0;

};

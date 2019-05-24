#pragma once

#include "Polygon.h"
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
  TArray<FPolygon> RoadPolygons;
  float TotalRoadArea = 0;
  int X = 0;
  
  FVector RandRoadPoint() const;

  const FActorDefinition& RandWalkerActorDefinition() const;

};

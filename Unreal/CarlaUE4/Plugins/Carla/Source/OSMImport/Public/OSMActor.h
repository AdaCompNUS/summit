#pragma once

#include "ProceduralMeshComponent.h"
#include "OSMActor.generated.h"

UCLASS(hidecategories = (Physics))
class OSMIMPORT_API AOSMActor : public AActor
{
	GENERATED_BODY()

public: 

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	UProceduralMeshComponent* MeshComponent;
  
  AOSMActor(const FObjectInitializer& ObjectInitializer);
		
  void SetOSM(const FString& OSMPath);

private:

  UMaterial* MeshMaterial;
  
  static constexpr double EarthCircumference = 40075036.0;
  static constexpr double LatitudeLongitudeScale = EarthCircumference / 360.0; // meters per degree
	static constexpr double OSMToCentimetersScaleFactor = 100.0;
    
  static double ConvertLatitudeToMeters(double Latitude) {
      return -Latitude * LatitudeLongitudeScale;
  }
  
  static double ConvertLongitudeToMeters(double Longitude, double Latitude) {
    return Longitude * LatitudeLongitudeScale * FMath::Cos(FMath::DegreesToRadians(Latitude));
  }
   
  static FVector2D ConvertLatLongToMetersRelative(double Latitude, double Longitude, double RelativeToLatitude, double RelativeToLongitude) {
    // Applies Sanson-Flamsteed (sinusoidal) Projection (see http://www.progonos.com/furuti/MapProj/Normal/CartHow/HowSanson/howSanson.html)
    return FVector2D(
      (float)( ConvertLongitudeToMeters( Longitude, Latitude ) - ConvertLongitudeToMeters( RelativeToLongitude, Latitude ) ),
      (float)( ConvertLatitudeToMeters( Latitude ) - ConvertLatitudeToMeters( RelativeToLatitude ) ) );
  }
};

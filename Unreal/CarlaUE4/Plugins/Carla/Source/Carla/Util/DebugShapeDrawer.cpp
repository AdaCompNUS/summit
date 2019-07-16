// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Util/DebugShapeDrawer.h"

#include "DrawDebugHelpers.h"
#include "Components/LineBatchComponent.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/rpc/DebugShape.h>
#include <carla/rpc/String.h>
#include <compiler/enable-ue4-macros.h>

struct FShapeVisitor
{
  using Shape = carla::rpc::DebugShape;

  FShapeVisitor(UWorld &InWorld, FColor InColor, float InLifeTime, bool bInPersistentLines)
    : World(&InWorld),
      Color(InColor),
      LifeTime(InLifeTime),
      bPersistentLines(bInPersistentLines)
  {}

  void operator()(const Shape::Point &Point) const
  {
    World->PersistentLineBatcher->DrawPoint(
        Point.location,
        Color,
        1e2f * Point.size,
        DepthPriority,
        LifeTime);
  }

  void operator()(const Shape::Line &Line) const
  {
    World->PersistentLineBatcher->DrawLine(
        Line.begin,
        Line.end,
        Color,
        DepthPriority,
        1e2f * Line.thickness,
        LifeTime);
  }

  void operator()(const Shape::Arrow &Arrow) const
  {
    const auto Diff = Arrow.line.end - Arrow.line.begin;
    const FRotator LookAt = FRotationMatrix::MakeFromX(Diff).Rotator();
    const FTransform Transform = {LookAt, Arrow.line.begin};

    // Everything in centimeters
    const auto Dist = 1e2f * Diff.Length();
    const auto ArrowSize = 1e2f * Arrow.arrow_size;
    const auto ArrowTipDist = Dist - ArrowSize;
    const auto Thickness = 1e2f * Arrow.line.thickness;

    World->PersistentLineBatcher->DrawLines(TArray<FBatchedLine>({
        FBatchedLine(
            Arrow.line.begin,
            Arrow.line.end,
            Color,
            LifeTime,
            Thickness,
            DepthPriority),
        FBatchedLine(
            Transform.TransformPosition(FVector(ArrowTipDist, +ArrowSize, +ArrowSize)),
            Arrow.line.end,
            Color,
            LifeTime,
            Thickness,
            DepthPriority),
        FBatchedLine(
            Transform.TransformPosition(FVector(ArrowTipDist, +ArrowSize, -ArrowSize)),
            Arrow.line.end,
            Color,
            LifeTime,
            Thickness,
            DepthPriority),
        FBatchedLine(
            Transform.TransformPosition(FVector(ArrowTipDist, -ArrowSize, +ArrowSize)),
            Arrow.line.end,
            Color,
            LifeTime,
            Thickness,
            DepthPriority),
        FBatchedLine(
            Transform.TransformPosition(FVector(ArrowTipDist, -ArrowSize, -ArrowSize)),
            Arrow.line.end,
            Color,
            LifeTime,
            Thickness,
            DepthPriority)}));
  }

  void operator()(const Shape::Box &Box) const
  {
    const FVector Extent = 1e2f * FVector{Box.box.extent.x, Box.box.extent.y, Box.box.extent.z};
    const FTransform Transform = {FRotator(Box.rotation), Box.box.location};
    const auto Thickness = 1e2f * Box.thickness;

    FVector B[2], P, Q;
    B[0] = -Extent;
    B[1] =  Extent;

    for(int32 i = 0; i < 2; ++i)
    {
      for(int32 j = 0; j < 2; ++j)
      {
        P.X=B[i].X;
        Q.X=B[i].X;
        P.Y=B[j].Y;
        Q.Y=B[j].Y;
        P.Z=B[0].Z;
        Q.Z=B[1].Z;
        World->PersistentLineBatcher->DrawLine(
            Transform.TransformPosition(P),
            Transform.TransformPosition(Q),
            Color,
            DepthPriority,
            Thickness,
            LifeTime);

        P.Y=B[i].Y;
        Q.Y=B[i].Y;
        P.Z=B[j].Z;
        Q.Z=B[j].Z;
        P.X=B[0].X;
        Q.X=B[1].X;
        World->PersistentLineBatcher->DrawLine(
            Transform.TransformPosition(P),
            Transform.TransformPosition(Q),
            Color,
            DepthPriority,
            Thickness,
            LifeTime);

        P.Z=B[i].Z;
        Q.Z=B[i].Z;
        P.X=B[j].X;
        Q.X=B[j].X;
        P.Y=B[0].Y;
        Q.Y=B[1].Y;
        World->PersistentLineBatcher->DrawLine(
            Transform.TransformPosition(P),
            Transform.TransformPosition(Q),
            Color,
            DepthPriority,
            Thickness,
            LifeTime);
      }
    }
  }

  void operator()(const Shape::String &Str) const
  {
    auto PlayerController = UGameplayStatics::GetPlayerController(World, 0);
    if (PlayerController == nullptr)
    {
      UE_LOG(LogCarla, Error, TEXT("Can't find player controller!"));
      return;
    }
    ACarlaHUD *Hud = Cast<ACarlaHUD>(PlayerController->GetHUD());
    Hud->AddHUDString(carla::rpc::ToFString(Str.text), Str.location, Color, LifeTime);
  }

private:

  UWorld *World;

  FColor Color;

  float LifeTime;

  bool bPersistentLines;

  uint8 DepthPriority = SDPG_World;
};

void FDebugShapeDrawer::Draw(const carla::rpc::DebugShape &Shape)
{
  auto Visitor = FShapeVisitor(World, Shape.color, Shape.life_time, Shape.persistent_lines);
  boost::apply_visitor(Visitor, Shape.primitive);
}

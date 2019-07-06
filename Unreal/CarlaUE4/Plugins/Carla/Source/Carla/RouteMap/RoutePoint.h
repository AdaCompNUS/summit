#pragma once

class FRoutePoint {
public:
  long long GetID() const { return ID; }
  const FVector2D& GetPosition() const { return Position; }
  
  FRoutePoint(long long ID, const FVector2D& Position) : ID(ID), Position(Position) { }

private:
  long long ID;
  FVector2D Position;
};

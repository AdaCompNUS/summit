#pragma once

class FRoutePoint {
public:
  long long GetID() const { return ID; }
  float GetOffset() const { return Offset; }

  FRoutePoint() { }
  FRoutePoint(long long ID, float Offset) : ID(ID), Offset(Offset) { }

private:
  long long ID;
  float Offset;
};

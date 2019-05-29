#pragma once

class FOccupancyGrid {

public:

  FOccupancyGrid() : Data(false, 0) { }

  FOccupancyGrid(int Width, int Height)
    : Width(Width), Height(Height), Data(false, Width * Height) { }

  int GetWidth() const { return Width; }

  int GetHeight() const { return Height; }

  int Num() const { return Data.Num(); }

  FBitReference operator()(int X, int Y) { return Data[Y * Width + X]; }

  FConstBitReference operator()(int X, int Y) const { return Data[Y * Width + X]; }

  FBitReference operator()(const FIntPoint& Point) { return Data[Point.Y * Width + Point.X]; }

  FConstBitReference operator()(const FIntPoint& Point) const { return Data[Point.Y * Width + Point.X]; }

private:

  int Width;
  int Height;
  TBitArray<FDefaultBitArrayAllocator> Data;

};

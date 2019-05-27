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

private:

  int Width;
  int Height;
  TBitArray<FDefaultBitArrayAllocator> Data;

};

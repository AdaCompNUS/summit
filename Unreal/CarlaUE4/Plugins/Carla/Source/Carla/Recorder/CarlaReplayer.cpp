// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaReplayer.h"
#include "CarlaRecorder.h"

#include <ctime>
#include <sstream>

// structure to save replaying info when need to load a new map (static member by now)
CarlaReplayer::PlayAfterLoadMap CarlaReplayer::Autoplay { false, "", "", 0.0, 0.0, 0, 1.0 };

void CarlaReplayer::Stop(bool bKeepActors)
{
  if (Enabled)
  {
    Enabled = false;

    // destroy actors if event was recorded?
    if (!bKeepActors)
    {
      ProcessToTime(TotalTime, false);
    }

    // callback
    Helper.ProcessReplayerFinish(bKeepActors);
  }

  File.close();
}

bool CarlaReplayer::ReadHeader()
{
  if (File.eof())
  {
    return false;
  }

  ReadValue<char>(File, Header.Id);
  ReadValue<uint32_t>(File, Header.Size);

  return true;
}

void CarlaReplayer::SkipPacket(void)
{
  File.seekg(Header.Size, std::ios::cur);
}

void CarlaReplayer::Rewind(void)
{
  CurrentTime = 0.0f;
  TotalTime = 0.0f;
  TimeToStop = 0.0f;

  File.clear();
  File.seekg(0, std::ios::beg);

  // mark as header as invalid to force reload a new one next time
  Frame.Elapsed = -1.0f;
  Frame.DurationThis = 0.0f;

  MappedId.clear();

  // read geneal Info
  RecInfo.Read(File);
}

// read last frame in File and return the Total time recorded
double CarlaReplayer::GetTotalTime(void)
{
  std::streampos Current = File.tellg();

  // parse only frames
  while (File)
  {
    // get header
    if (!ReadHeader())
    {
      break;
    }

    // check for a frame packet
    switch (Header.Id)
    {
      case static_cast<char>(CarlaRecorderPacketId::FrameStart):
        Frame.Read(File);
        break;
      default:
        SkipPacket();
        break;
    }
  }

  File.clear();
  File.seekg(Current, std::ios::beg);
  return Frame.Elapsed;
}

std::string CarlaReplayer::ReplayFile(std::string Filename, double TimeStart, double Duration, uint32_t ThisFollowId)
{
  std::stringstream Info;
  std::string s;

  // check to stop if we are replaying another
  if (Enabled)
  {
    Stop();
  }

  // get the final path + filename
  std::string Filename2 = GetRecorderFilename(Filename);

  Info << "Replaying File: " << Filename2 << std::endl;

  // try to open
  File.open(Filename2, std::ios::binary);
  if (!File.is_open())
  {
    Info << "File " << Filename2 << " not found on server\n";
    Stop();
    return Info.str();
  }

  // from start
  Rewind();

  // check to load map if different
  if (Episode->GetMapName() != RecInfo.Mapfile)
  {
    if (!Episode->LoadNewEpisode(RecInfo.Mapfile))
    {
      Info << "Could not load mapfile " << TCHAR_TO_UTF8(*RecInfo.Mapfile) << std::endl;
      Stop();
      return Info.str();
    }
    Info << "Loading map " << TCHAR_TO_UTF8(*RecInfo.Mapfile) << std::endl;
    Info << "Replayer will start after map is loaded..." << std::endl;

    // prepare autoplay after map is loaded
    Autoplay.Enabled = true;
    Autoplay.Filename = Filename2;
    Autoplay.Mapfile = RecInfo.Mapfile;
    Autoplay.TimeStart = TimeStart;
    Autoplay.Duration = Duration;
    Autoplay.FollowId = ThisFollowId;
    Autoplay.TimeFactor = TimeFactor;
  }

  // get Total time of recorder
  TotalTime = GetTotalTime();
  Info << "Total time recorded: " << TotalTime << std::endl;

  // set time to start replayer
  if (TimeStart < 0.0f)
  {
    TimeStart = TotalTime + TimeStart;
    if (TimeStart < 0.0f)
      TimeStart = 0.0f;
  }

  // set time to stop replayer
  if (Duration > 0.0f)
    TimeToStop = TimeStart + Duration;
  else
    TimeToStop = TotalTime;

  Info << "Replaying from " << TimeStart << " s - " << TimeToStop << " s (" << TotalTime << " s) at " <<
      std::setprecision(1) << std::fixed << TimeFactor << "x" << std::endl;

  // set the follow Id
  FollowId = ThisFollowId;

  // if we don't need to load a new map, then start
  if (!Autoplay.Enabled)
  {
    // process all events until the time
    ProcessToTime(TimeStart, true);
    // mark as enabled
    Enabled = true;
  }

  return Info.str();
}

void CarlaReplayer::CheckPlayAfterMapLoaded(void)
{

  // check if the autoplay is enabled (means waiting until map is loaded)
  if (!Autoplay.Enabled)
    return;

  // disable
  Autoplay.Enabled = false;

  // check to stop if we are replaying another
  if (Enabled)
  {
    Stop();
  }

  // try to open
  File.open(Autoplay.Filename, std::ios::binary);
  if (!File.is_open())
  {
    return;
  }

  // from start
  Rewind();

  // get Total time of recorder
  TotalTime = GetTotalTime();

  // set time to start replayer
  double TimeStart = Autoplay.TimeStart;
  if (TimeStart < 0.0f)
  {
    TimeStart = TotalTime + Autoplay.TimeStart;
    if (TimeStart < 0.0f)
      TimeStart = 0.0f;
  }

  // set time to stop replayer
  if (Autoplay.Duration > 0.0f)
    TimeToStop = TimeStart + Autoplay.Duration;
  else
    TimeToStop = TotalTime;

  // set the follow Id
  FollowId = Autoplay.FollowId;

  // apply time factor
  TimeFactor = Autoplay.TimeFactor;

  // process all events until the time
  ProcessToTime(TimeStart, true);

  // mark as enabled
  Enabled = true;
}

void CarlaReplayer::ProcessToTime(double Time, bool IsFirstTime)
{
  double Per = 0.0f;
  double NewTime = CurrentTime + Time;
  bool bFrameFound = false;
  bool bExitAtNextFrame = false;
  bool bExitLoop = false;

  // check if we are in the right frame
  if (NewTime >= Frame.Elapsed && NewTime < Frame.Elapsed + Frame.DurationThis)
  {
    Per = (NewTime - Frame.Elapsed) / Frame.DurationThis;
    bFrameFound = true;
    bExitLoop = true;
  }

  // process all frames until time we want or end
  while (!File.eof() && !bExitLoop)
  {
    // get header
    ReadHeader();

    // check for a frame packet
    switch (Header.Id)
    {
      // frame
      case static_cast<char>(CarlaRecorderPacketId::FrameStart):
        // only read if we are not in the right frame
        Frame.Read(File);
        // check if target time is in this frame
        if (NewTime < Frame.Elapsed + Frame.DurationThis)
        {
          Per = (NewTime - Frame.Elapsed) / Frame.DurationThis;
          bFrameFound = true;
        }
        break;

      // events add
      case static_cast<char>(CarlaRecorderPacketId::EventAdd):
        ProcessEventsAdd();
        break;

      // events del
      case static_cast<char>(CarlaRecorderPacketId::EventDel):
        ProcessEventsDel();
        break;

      // events parent
      case static_cast<char>(CarlaRecorderPacketId::EventParent):
        ProcessEventsParent();
        break;

      // collisions
      case static_cast<char>(CarlaRecorderPacketId::Collision):
        SkipPacket();
        break;

      // positions
      case static_cast<char>(CarlaRecorderPacketId::Position):
        if (bFrameFound)
          ProcessPositions(IsFirstTime);
        else
          SkipPacket();
        break;

      // states
      case static_cast<char>(CarlaRecorderPacketId::State):
        if (bFrameFound)
          ProcessStates();
        else
          SkipPacket();
        break;

      // vehicle animation
      case static_cast<char>(CarlaRecorderPacketId::AnimVehicle):
        if (bFrameFound)
          ProcessAnimVehicle();
        else
          SkipPacket();
        break;

      // walker animation
      case static_cast<char>(CarlaRecorderPacketId::AnimWalker):
        if (bFrameFound)
          ProcessAnimWalker();
        else
          SkipPacket();
        break;

      // frame end
      case static_cast<char>(CarlaRecorderPacketId::FrameEnd):
        if (bFrameFound)
          bExitLoop = true;
        break;

      // unknown packet, just skip
      default:
        // skip packet
        SkipPacket();
        break;

    }
  }

  // update all positions
  if (Enabled && bFrameFound)
  {
    UpdatePositions(Per, Time);
  }

  // save current time
  CurrentTime = NewTime;

  // stop replay?
  if (CurrentTime >= TimeToStop)
  {
    // keep actors in scene and let them continue with autopilot
    Stop(true);
  }
}

void CarlaReplayer::ProcessEventsAdd(void)
{
  uint16_t i, Total;
  CarlaRecorderEventAdd EventAdd;

  // process creation events
  ReadValue<uint16_t>(File, Total);
  for (i = 0; i < Total; ++i)
  {
    EventAdd.Read(File);

    // auto Result = CallbackEventAdd(
    auto Result = Helper.ProcessReplayerEventAdd(
        EventAdd.Location,
        EventAdd.Rotation,
        std::move(EventAdd.Description),
        EventAdd.DatabaseId);

    switch (Result.first)
    {
      // actor not created
      case 0:
        UE_LOG(LogCarla, Log, TEXT("actor could not be created"));
        break;

      // actor created but with different id
      case 1:
        // mapping id (recorded Id is a new Id in replayer)
        MappedId[EventAdd.DatabaseId] = Result.second;
        break;

      // actor reused from existing
      case 2:
        // mapping id (say desired Id is mapped to what)
        MappedId[EventAdd.DatabaseId] = Result.second;
        break;
    }
  }
}

void CarlaReplayer::ProcessEventsDel(void)
{
  uint16_t i, Total;
  CarlaRecorderEventDel EventDel;

  // process destroy events
  ReadValue<uint16_t>(File, Total);
  for (i = 0; i < Total; ++i)
  {
    EventDel.Read(File);
    Helper.ProcessReplayerEventDel(MappedId[EventDel.DatabaseId]);
    MappedId.erase(EventDel.DatabaseId);
  }
}

void CarlaReplayer::ProcessEventsParent(void)
{
  uint16_t i, Total;
  CarlaRecorderEventParent EventParent;
  std::stringstream Info;

  // process parenting events
  ReadValue<uint16_t>(File, Total);
  for (i = 0; i < Total; ++i)
  {
    EventParent.Read(File);
    Helper.ProcessReplayerEventParent(MappedId[EventParent.DatabaseId], MappedId[EventParent.DatabaseIdParent]);
  }
}

void CarlaReplayer::ProcessStates(void)
{
  uint16_t i, Total;
  CarlaRecorderStateTrafficLight StateTrafficLight;
  std::stringstream Info;

  // read Total traffic light states
  ReadValue<uint16_t>(File, Total);
  for (i = 0; i < Total; ++i)
  {
    StateTrafficLight.Read(File);

    StateTrafficLight.DatabaseId = MappedId[StateTrafficLight.DatabaseId];
    if (!Helper.ProcessReplayerStateTrafficLight(StateTrafficLight))
    {
      UE_LOG(LogCarla,
          Log,
          TEXT("callback state traffic light %d called but didn't work"),
          StateTrafficLight.DatabaseId);
    }
  }
}

void CarlaReplayer::ProcessAnimVehicle(void)
{
  uint16_t i, Total;
  CarlaRecorderAnimVehicle Vehicle;
  std::stringstream Info;

  // read Total Vehicles
  ReadValue<uint16_t>(File, Total);
  for (i = 0; i < Total; ++i)
  {
    Vehicle.Read(File);
    Vehicle.DatabaseId = MappedId[Vehicle.DatabaseId];
    Helper.ProcessReplayerAnimVehicle(Vehicle);
  }
}

void CarlaReplayer::ProcessAnimWalker(void)
{
  uint16_t i, Total;
  CarlaRecorderAnimWalker Walker;
  std::stringstream Info;

  // read Total walkers
  ReadValue<uint16_t>(File, Total);
  for (i = 0; i < Total; ++i)
  {
    Walker.Read(File);
    Walker.DatabaseId = MappedId[Walker.DatabaseId];
    Helper.ProcessReplayerAnimWalker(Walker);
  }
}

void CarlaReplayer::ProcessPositions(bool IsFirstTime)
{
  uint16_t i, Total;

  // save current as previous
  PrevPos = std::move(CurrPos);

  // read all positions
  ReadValue<uint16_t>(File, Total);
  CurrPos.clear();
  CurrPos.reserve(Total);
  for (i = 0; i < Total; ++i)
  {
    CarlaRecorderPosition Pos;
    Pos.Read(File);
    // assign mapped Id
    auto NewId = MappedId.find(Pos.DatabaseId);
    if (NewId != MappedId.end())
    {
      Pos.DatabaseId = NewId->second;
    }
    else
      UE_LOG(LogCarla, Log, TEXT("Actor not found when trying to move from replayer (id. %d)"), Pos.DatabaseId);
    CurrPos.push_back(std::move(Pos));
  }

  // check to copy positions the first time
  if (IsFirstTime)
  {
    PrevPos.clear();
  }
}

void CarlaReplayer::UpdatePositions(double Per, double DeltaTime)
{
  unsigned int i;
  uint32_t NewFollowId = 0;
  std::unordered_map<int, int> TempMap;

  // map the id of all previous positions to its index
  for (i = 0; i < PrevPos.size(); ++i)
  {
    TempMap[PrevPos[i].DatabaseId] = i;
  }

  // get the Id of the actor to follow
  if (FollowId != 0)
  {
    auto NewId = MappedId.find(FollowId);
    if (NewId != MappedId.end())
    {
      NewFollowId = NewId->second;
    }
  }

  // go through each actor and update
  for (i = 0; i < CurrPos.size(); ++i)
  {
    // check if exist a previous position
    auto Result = TempMap.find(CurrPos[i].DatabaseId);
    if (Result != TempMap.end())
    {
      // check if time factor is high
      if (TimeFactor >= 2.0)
        // assign first position
        InterpolatePosition(PrevPos[Result->second], CurrPos[i], 0.0, DeltaTime);
      else
        // interpolate
        InterpolatePosition(PrevPos[Result->second], CurrPos[i], Per, DeltaTime);
    }
    else
    {
      // assign last position (we don't have previous one)
      InterpolatePosition(CurrPos[i], CurrPos[i], 0.0, DeltaTime);
    }

    // move the camera to follow this actor if required
    if (NewFollowId != 0)
    {
      if (NewFollowId == CurrPos[i].DatabaseId)
        Helper.SetCameraPosition(NewFollowId, FVector(-1000, 0, 500), FQuat::MakeFromEuler({0, -25, 0}));
    }
  }
}

// interpolate a position (transform, velocity...)
void CarlaReplayer::InterpolatePosition(
    const CarlaRecorderPosition &Pos1,
    const CarlaRecorderPosition &Pos2,
    double Per,
    double DeltaTime)
{
  // call the callback
  Helper.ProcessReplayerPosition(Pos1, Pos2, Per, DeltaTime);
}

// tick for the replayer
void CarlaReplayer::Tick(float Delta)
{
  // check if there are events to process
  if (Enabled)
  {
    ProcessToTime(Delta * TimeFactor, false);
  }
}

// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/PythonUtil.h>
#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/World.h>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace carla {
namespace client {

  std::ostream &operator<<(std::ostream &out, const ActorList &actors) {
    return PrintList(out, actors);
  }

  std::ostream &operator<<(std::ostream &out, const Timestamp &timestamp) {
    out << "Timestamp(frame_count=" << timestamp.frame_count
        << ",elapsed_seconds=" << timestamp.elapsed_seconds
        << ",delta_seconds=" << timestamp.delta_seconds
        << ",platform_timestamp=" << timestamp.platform_timestamp << ')';
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const World &world) {
    out << "World(id=" << world.GetId() << ')';
    return out;
  }

} // namespace client
} // namespace carla

namespace carla {
namespace rpc {

  std::ostream &operator<<(std::ostream &out, const EpisodeSettings &settings) {
    auto BoolToStr = [](bool b) { return b ? "True" : "False"; };
    out << "WorldSettings(synchronous_mode=" << BoolToStr(settings.synchronous_mode)
        << ",no_rendering_mode=" << BoolToStr(settings.no_rendering_mode) << ')';
    return out;
  }

} // namespace rpc
} // namespace carla

static auto WaitForTick(const carla::client::World &world, double seconds) {
  carla::PythonUtil::ReleaseGIL unlock;
  return world.WaitForTick(TimeDurationFromSeconds(seconds));
}

static void OnTick(carla::client::World &self, boost::python::object callback) {
  self.OnTick(MakeCallback(std::move(callback)));
}

static auto GetActorsById(carla::client::World &self, const boost::python::list &actor_ids) {
  std::vector<carla::ActorId> ids{
      boost::python::stl_input_iterator<carla::ActorId>(actor_ids),
      boost::python::stl_input_iterator<carla::ActorId>()};
  carla::PythonUtil::ReleaseGIL unlock;
  return self.GetActors(ids);
}

void export_world() {
  using namespace boost::python;
  namespace cc = carla::client;
  namespace cg = carla::geom;
  namespace cr = carla::rpc;

  class_<cc::Timestamp>("Timestamp")
    .def(init<size_t, double, double, double>(
        (arg("frame_count")=0u,
         arg("elapsed_seconds")=0.0,
         arg("delta_seconds")=0.0,
         arg("platform_timestamp")=0.0)))
    .def_readwrite("frame_count", &cc::Timestamp::frame_count)
    .def_readwrite("elapsed_seconds", &cc::Timestamp::elapsed_seconds)
    .def_readwrite("delta_seconds", &cc::Timestamp::delta_seconds)
    .def_readwrite("platform_timestamp", &cc::Timestamp::platform_timestamp)
    .def("__eq__", &cc::Timestamp::operator==)
    .def("__ne__", &cc::Timestamp::operator!=)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::ActorList, boost::shared_ptr<cc::ActorList>>("ActorList", no_init)
    .def("find", &cc::ActorList::Find, (arg("id")))
    .def("filter", &cc::ActorList::Filter, (arg("wildcard_pattern")))
    .def("__getitem__", &cc::ActorList::at)
    .def("__len__", &cc::ActorList::size)
    .def("__iter__", range(&cc::ActorList::begin, &cc::ActorList::end))
    .def(self_ns::str(self_ns::self))
  ;

  class_<cr::EpisodeSettings>("WorldSettings")
    .def(init<bool, bool>(
        (arg("synchronous_mode")=false,
         arg("no_rendering_mode")=false)))
    .def_readwrite("synchronous_mode", &cr::EpisodeSettings::synchronous_mode)
    .def_readwrite("no_rendering_mode", &cr::EpisodeSettings::no_rendering_mode)
    .def("__eq__", &cc::Timestamp::operator==)
    .def("__ne__", &cc::Timestamp::operator!=)
    .def(self_ns::str(self_ns::self))
  ;

  enum_<cr::AttachmentType>("AttachmentType")
    .value("Rigid", cr::AttachmentType::Rigid)
    .value("SpringArm", cr::AttachmentType::SpringArm)
  ;

#define SPAWN_ACTOR_WITHOUT_GIL(fn) +[]( \
        cc::World &self, \
        const cc::ActorBlueprint &blueprint, \
        const cg::Transform &transform, \
        cc::Actor *parent, \
        cr::AttachmentType attachment_type) { \
      carla::PythonUtil::ReleaseGIL unlock; \
      return self.fn(blueprint, transform, parent, attachment_type); \
    }, \
    ( \
      arg("blueprint"), \
      arg("transform"), \
      arg("attach_to")=carla::SharedPtr<cc::Actor>(), \
      arg("attachment_type")=cr::AttachmentType::Rigid)

  class_<cc::World>("World", no_init)
    .add_property("id", &cc::World::GetId)
    .add_property("debug", &cc::World::MakeDebugHelper)
    .def("get_blueprint_library", CONST_CALL_WITHOUT_GIL(cc::World, GetBlueprintLibrary))
    .def("get_map", CONST_CALL_WITHOUT_GIL(cc::World, GetMap))
    .def("get_spectator", CONST_CALL_WITHOUT_GIL(cc::World, GetSpectator))
    .def("get_settings", CONST_CALL_WITHOUT_GIL(cc::World, GetSettings))
    .def("apply_settings", &cc::World::ApplySettings)
    .def("get_weather", CONST_CALL_WITHOUT_GIL(cc::World, GetWeather))
    .def("set_weather", &cc::World::SetWeather)
    .def("get_actors", CONST_CALL_WITHOUT_GIL(cc::World, GetActors))
    .def("get_actors", &GetActorsById, (arg("actor_ids")))
    .def("spawn_actor", SPAWN_ACTOR_WITHOUT_GIL(SpawnActor))
    .def("try_spawn_actor", SPAWN_ACTOR_WITHOUT_GIL(TrySpawnActor))
    .def("wait_for_tick", &WaitForTick, (arg("seconds")=10.0))
    .def("on_tick", &OnTick, (arg("callback")))
    .def("tick", &cc::World::Tick)
    .def(self_ns::str(self_ns::self))
  ;

#undef SPAWN_ACTOR_WITHOUT_GIL

  class_<cc::DebugHelper>("DebugHelper", no_init)
    .def("draw_point", &cc::DebugHelper::DrawPoint,
        (arg("location"),
         arg("size")=0.1f,
         arg("color")=cc::DebugHelper::Color(255u, 0u, 0u),
         arg("life_time")=-1.0f,
         arg("persistent_lines")=true))
    .def("draw_line", &cc::DebugHelper::DrawLine,
        (arg("begin"),
         arg("end"),
         arg("thickness")=0.1f,
         arg("color")=cc::DebugHelper::Color(255u, 0u, 0u),
         arg("life_time")=-1.0f,
         arg("persistent_lines")=true))
    .def("draw_arrow", &cc::DebugHelper::DrawArrow,
        (arg("begin"),
         arg("end"),
         arg("thickness")=0.1f,
         arg("arrow_size")=0.1f,
         arg("color")=cc::DebugHelper::Color(255u, 0u, 0u),
         arg("life_time")=-1.0f,
         arg("persistent_lines")=true))
    .def("draw_box", &cc::DebugHelper::DrawBox,
        (arg("box"),
         arg("rotation"),
         arg("thickness")=0.1f,
         arg("color")=cc::DebugHelper::Color(255u, 0u, 0u),
         arg("life_time")=-1.0f,
         arg("persistent_lines")=true))
    .def("draw_string", &cc::DebugHelper::DrawString,
        (arg("location"),
         arg("text"),
         arg("draw_shadow")=false,
         arg("color")=cc::DebugHelper::Color(255u, 0u, 0u),
         arg("life_time")=-1.0f,
         arg("persistent_lines")=true))
  ;
}

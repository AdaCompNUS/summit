#include <carla/geom/AABB2D.h>
#include <carla/aabb/AABBMap.h>
#include <cstdint>

void export_aabb() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::aabb;

  class_<AABBMap>("AABBMap", no_init)
    .def(init<>())
    .def(init<const std::vector<geom::AABB2D>&>())
    .def("__init__", make_constructor(
          +[](const list& aabbs_py) {
            std::vector<geom::AABB2D> aabbs{
              stl_input_iterator<carla::geom::AABB2D>(aabbs_py),
              stl_input_iterator<carla::geom::AABB2D>()};
            return boost::shared_ptr<AABBMap>(new AABBMap(aabbs));
          }))
    .def("__len__", &AABBMap::Count)
    .def("insert", &AABBMap::Insert)
    .def("intersects", &AABBMap::Intersects)
  ;
}

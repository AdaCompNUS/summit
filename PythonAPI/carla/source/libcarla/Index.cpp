#include <carla/geom/Triangle.h>
#include <carla/index/TriangleIndex.h>

static auto MakeTriangleIndex(const boost::python::list& triangles) {
  std::vector<carla::geom::Triangle> _triangles{
      boost::python::stl_input_iterator<carla::geom::Triangle>(triangles),
      boost::python::stl_input_iterator<carla::geom::Triangle>()};
  return boost::shared_ptr<carla::index::TriangleIndex>(
      new carla::index::TriangleIndex(_triangles));
}

void export_index() {
  using namespace boost::python;

  class_<carla::index::TriangleIndex>("TriangleIndex", no_init)
    .def("__init__", make_constructor(&MakeTriangleIndex))
    .def("__len__", &carla::index::TriangleIndex::Size)
    .def("query_intersect", &carla::index::TriangleIndex::QueryIntersect)
  ;
}

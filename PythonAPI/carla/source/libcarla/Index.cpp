#include <carla/geom/Triangle2D.h>
#include <carla/index/Triangle2DIndex.h>

static auto MakeTriangle2DIndex(const boost::python::list& triangles) {
  std::vector<carla::geom::Triangle2D> _triangles{
      boost::python::stl_input_iterator<carla::geom::Triangle2D>(triangles),
      boost::python::stl_input_iterator<carla::geom::Triangle2D>()};
  return boost::shared_ptr<carla::index::Triangle2DIndex>(
      new carla::index::Triangle2DIndex(_triangles));
}

void export_index() {
  using namespace boost::python;

  class_<carla::index::Triangle2DIndex>("Triangle2DIndex", no_init)
    .def("__init__", make_constructor(&MakeTriangle2DIndex))
    .def("__len__", &carla::index::Triangle2DIndex::Size)
    .def("query_intersect", &carla::index::Triangle2DIndex::QueryIntersect)
  ;
}

#include <carla/geom/Segment2D.h>
#include <carla/segments/SegmentMap.h>
#include <boost/python/numpy.hpp>
#include <cstdint>

void export_segments() {
  using namespace boost::python;
  using namespace carla;
  using namespace carla::segments;

  class_<SegmentMap>("SegmentMap", no_init)
    .def(init<>())
    .def(init<const std::vector<geom::Segment2D>&>())
    .def("__init__", make_constructor(
          +[](const list& segments_py) {
            std::vector<geom::Segment2D> segments{
              stl_input_iterator<carla::geom::Segment2D>(segments_py),
              stl_input_iterator<carla::geom::Segment2D>()};
            return boost::shared_ptr<SegmentMap>(new SegmentMap(segments));
          }))
    .def("union", &SegmentMap::Union)
    .def("difference", &SegmentMap::Difference)
    .def("intersection", &SegmentMap::Intersection)
    .def("rand_point", &SegmentMap::RandPoint)
  ;
}

// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/geom/BoundingBox.h>
#include <carla/geom/GeoLocation.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Vector2D.h>
#include <carla/geom/Vector3D.h>

#include <boost/python/implicit.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <ostream>

namespace carla {
namespace geom {

  template <typename T>
  static void WriteVector2D(std::ostream &out, const char *name, const T &vector2D) {
    out << name
        << "(x=" << std::to_string(vector2D.x)
        << ", y=" << std::to_string(vector2D.y) << ')';
  }

  template <typename T>
  static void WriteVector3D(std::ostream &out, const char *name, const T &vector3D) {
    out << name
        << "(x=" << std::to_string(vector3D.x)
        << ", y=" << std::to_string(vector3D.y)
        << ", z=" << std::to_string(vector3D.z) << ')';
  }

  std::ostream &operator<<(std::ostream &out, const Vector2D &vector2D) {
    WriteVector2D(out, "Vector2D", vector2D);
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const Vector3D &vector3D) {
    WriteVector3D(out, "Vector3D", vector3D);
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const Location &location) {
    WriteVector3D(out, "Location", location);
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const Rotation &rotation) {
    out << "Rotation(pitch=" << std::to_string(rotation.pitch)
        << ", yaw=" << std::to_string(rotation.yaw)
        << ", roll=" << std::to_string(rotation.roll) << ')';
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const Transform &transform) {
    out << "Transform(" << transform.location << ", " << transform.rotation << ')';
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const BoundingBox &box) {
    out << "BoundingBox(" << box.location << ", ";
    WriteVector3D(out, "Extent", box.extent);
    out << ')';
    return out;
  }

  std::ostream &operator<<(std::ostream &out, const GeoLocation &geo_location) {
    out << "GeoLocation(latitude=" << std::to_string(geo_location.latitude)
        << ", longitude=" << std::to_string(geo_location.longitude)
        << ", altitude=" << std::to_string(geo_location.altitude) << ')';
    return out;
  }

} // namespace geom
} // namespace carla

static void TransformList(const carla::geom::Transform &self, boost::python::list &list) {
  auto length = boost::python::len(list);
   for (auto i = 0u; i < length; ++i) {
    self.TransformPoint(boost::python::extract<carla::geom::Vector3D &>(list[i]));
  }
}

void export_geom() {
  using namespace boost::python;
  namespace cg = carla::geom;
  class_<std::vector<cg::Vector2D>>("vector_of_vector2D")
      .def(boost::python::vector_indexing_suite<std::vector<cg::Vector2D>>())
      .def(self_ns::str(self_ns::self))
  ;

  class_<cg::Vector2D>("Vector2D")
    .def(init<float, float>((arg("x")=0.0f, arg("y")=0.0f)))
    .def_readwrite("x", &cg::Vector2D::x)
    .def_readwrite("y", &cg::Vector2D::y)
    .def("__eq__", &cg::Vector2D::operator==)
    .def("__ne__", &cg::Vector2D::operator!=)
    .def(self += self)
    .def(self + self)
    .def(self -= self)
    .def(self - self)
    .def(self *= double())
    .def(self * double())
    .def(double() * self)
    .def(self /= double())
    .def(self / double())
    .def(double() / self)
    .def(self_ns::str(self_ns::self))
  ;

  implicitly_convertible<cg::Vector3D, cg::Location>();
  implicitly_convertible<cg::Location, cg::Vector3D>();

  class_<cg::Vector3D>("Vector3D")
    .def(init<float, float, float>((arg("x")=0.0f, arg("y")=0.0f, arg("z")=0.0f)))
    .def(init<const cg::Location &>((arg("rhs"))))
    .def_readwrite("x", &cg::Vector3D::x)
    .def_readwrite("y", &cg::Vector3D::y)
    .def_readwrite("z", &cg::Vector3D::z)
    .def("__eq__", &cg::Vector3D::operator==)
    .def("__ne__", &cg::Vector3D::operator!=)
    .def(self += self)
    .def(self + self)
    .def(self -= self)
    .def(self - self)
    .def(self *= double())
    .def(self * double())
    .def(double() * self)
    .def(self /= double())
    .def(self / double())
    .def(double() / self)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cg::Location, bases<cg::Vector3D>>("Location")
    .def(init<float, float, float>((arg("x")=0.0f, arg("y")=0.0f, arg("z")=0.0f)))
    .def(init<const cg::Vector3D &>((arg("rhs"))))
    .add_property("x", +[](const cg::Location &self) { return self.x; }, +[](cg::Location &self, float x) { self.x = x; })
    .add_property("y", +[](const cg::Location &self) { return self.y; }, +[](cg::Location &self, float y) { self.y = y; })
    .add_property("z", +[](const cg::Location &self) { return self.z; }, +[](cg::Location &self, float z) { self.z = z; })
    .def("distance", &cg::Location::Distance, (arg("location")))
    .def("__eq__", &cg::Location::operator==)
    .def("__ne__", &cg::Location::operator!=)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cg::Rotation>("Rotation")
    .def(init<float, float, float>((arg("pitch")=0.0f, arg("yaw")=0.0f, arg("roll")=0.0f)))
    .def_readwrite("pitch", &cg::Rotation::pitch)
    .def_readwrite("yaw", &cg::Rotation::yaw)
    .def_readwrite("roll", &cg::Rotation::roll)
    .def("get_forward_vector", &cg::Rotation::GetForwardVector)
    .def("__eq__", &cg::Rotation::operator==)
    .def("__ne__", &cg::Rotation::operator!=)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cg::Transform>("Transform")
    .def(init<cg::Location, cg::Rotation>(
        (arg("location")=cg::Location(), arg("rotation")=cg::Rotation())))
    .def_readwrite("location", &cg::Transform::location)
    .def_readwrite("rotation", &cg::Transform::rotation)
    .def("transform", &TransformList)
    .def("transform", +[](const cg::Transform &self, cg::Vector3D &location) {
      self.TransformPoint(location);
      return location;
    }, arg("in_point"))
    .def("get_forward_vector", &cg::Transform::GetForwardVector)
    .def("__eq__", &cg::Transform::operator==)
    .def("__ne__", &cg::Transform::operator!=)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cg::BoundingBox>("BoundingBox")
    .def(init<cg::Location, cg::Vector3D>(
        (arg("location")=cg::Location(), arg("extent")=cg::Vector3D())))
    .def_readwrite("location", &cg::BoundingBox::location)
    .def_readwrite("extent", &cg::BoundingBox::extent)
    .def("__eq__", &cg::BoundingBox::operator==)
    .def("__ne__", &cg::BoundingBox::operator!=)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cg::GeoLocation>("GeoLocation")
    .def(init<double, double, double>((arg("latitude")=0.0, arg("longitude")=0.0, arg("altitude")=0.0)))
    .def_readwrite("latitude", &cg::GeoLocation::latitude)
    .def_readwrite("longitude", &cg::GeoLocation::longitude)
    .def_readwrite("altitude", &cg::GeoLocation::altitude)
    .def("__eq__", &cg::GeoLocation::operator==)
    .def("__ne__", &cg::GeoLocation::operator!=)
    .def(self_ns::str(self_ns::self))
  ;
}

#include "LandmarkMap.h"

#include <osmium/handler.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/visitor.hpp>
#include <osmium/io/any_input.hpp>

namespace carla {
namespace landmark {

LandmarkMap LandmarkMap::Load(const std::string& file) {  
  osmium::io::File input_file{file};
  osmium::io::Reader reader{input_file};
  struct CountHandler : public osmium::handler::Handler {
    /*
    void node(const osmium::Node& node) noexcept {

    }

    void way(const osmium::Way& way) noexcept {
    }

    void relation(const osmium::Relation& relation) noexcept {
    }
    */
  } handler;
  osmium::apply(reader, handler);
  
  LandmarkMap landmark_map;
  return landmark_map;
}

}
}

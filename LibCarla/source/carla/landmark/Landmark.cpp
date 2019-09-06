#include "Landmark.h"

#include <osmium/geom/mercator_projection.hpp>
#include <osmium/handler.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/visitor.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>

namespace carla {
namespace landmark {

std::vector<occupancy::OccupancyMap> Landmark::Load(const std::string& file, const geom::Vector2D& offset) {  
  osmium::io::File input_file{file};
  osmium::io::Reader reader{input_file, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};

  using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
  using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;
  index_type index;
  location_handler_type location_handler{index};
  
  struct CountHandler : public osmium::handler::Handler {
    
    geom::Vector2D offset;
    std::vector<occupancy::OccupancyMap> landmarks;
    
    void way(const osmium::Way& way) noexcept {
      if (way.tags()["building"]) {
        std::vector<geom::Vector2D> outline;
        for (const osmium::NodeRef it_node : way.nodes()) {
          osmium::geom::Coordinates c = osmium::geom::lonlat_to_mercator(it_node.location());
          outline.emplace_back(
              static_cast<float>(c.y) + offset.y,  // Swap from Web Mercator -> CARLA.
              static_cast<float>(c.x) + offset.x); // Swap from Web Mercator -> CARLA.
        }
        landmarks.emplace_back(std::move(outline));
      }
    }

  } handler;
  handler.offset = offset;

  osmium::apply(reader, location_handler, handler);

  return handler.landmarks;
}

}
}

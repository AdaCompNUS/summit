#include "carla/geom/Vector2D.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace carla {
namespace sumonetwork {

struct Lane;
  
enum class Function {
  Normal,
  Internal,
  Connector,
  Crossing,
  WalkingArea
};

struct Edge {
  std::string id;
  std::string from;
  std::string to;
  int32_t priority;
  Function function;
  std::vector<Lane> lanes;
};

struct Lane {
  std::string id;
  uint32_t index;
  double speed;
  double length;
  std::vector<geom::Vector2D> shape;
};

struct Junction {
  std::string id;
  double x;
  double y;
  std::vector<std::string> inc_lanes;
  std::vector<std::string> int_lanes;
  std::vector<geom::Vector2D> shape;
};

struct Connection {
  std::string from;
  std::string to;
  uint32_t from_lane;
  uint32_t to_lane;
  std::string via;
};


class SumoNetwork {

public:

  static SumoNetwork Load(const std::string& data);
  
  const std::unordered_map<std::string, Edge>& Edges() const { return _edges; }
  const std::unordered_map<std::string, Junction>& Junctions() const { return _junctions; }
  const std::vector<Connection>& Connections() const { return _connections; }

private:

  std::unordered_map<std::string, Edge> _edges;
  std::unordered_map<std::string, Junction> _junctions;
  std::vector<Connection> _connections;

};

}
}

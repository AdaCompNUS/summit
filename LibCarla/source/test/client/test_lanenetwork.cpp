#include "test.h"

#include <boost/filesystem.hpp>
#include <carla/lanenetwork/LaneNetwork.h>
#include <string>
#include <iostream>

using namespace carla::lanenetwork;
using namespace boost::filesystem;

const std::string BASE_PATH = LIBCARLA_TEST_CONTENT_FOLDER "/LaneNetwork/";

TEST(lanenetwork, load_lane_network) {
  for (directory_iterator it(BASE_PATH); it != directory_iterator(); ++it) {
    std::cout << "Loading LaneNetwork from " << it->path().string() << std::endl;
    LaneNetwork lane_network = LaneNetwork::Load(it->path().string());
    std::cout << "Nodes = " << lane_network.Nodes().size() 
      << ", Roads = " << lane_network.Roads().size() 
      << ", Lanes = " << lane_network.Lanes().size() 
      << ", LaneConnections = " << lane_network.LaneConnections().size()
      << std::endl;
  }
}

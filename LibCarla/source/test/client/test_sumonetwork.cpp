#include "test.h"

#include <boost/filesystem.hpp>
#include <carla/sumonetwork/SumoNetwork.h>
#include <fstream>
#include <string>

using namespace carla::sumonetwork;
using namespace boost::filesystem;

const std::string BASE_PATH = LIBCARLA_TEST_CONTENT_FOLDER "/SUMO/";

TEST(sumonetwork, load_sumo_network) {
  for (directory_iterator it(BASE_PATH); it != directory_iterator(); ++it) {
    std::cout << "Loading SUMO Network from " << it->path().string() << std::endl;
    std::ifstream t(it->path().string());
    std::stringstream buffer;
    buffer << t.rdbuf();
    SumoNetwork::Load(buffer.str());
  }

}

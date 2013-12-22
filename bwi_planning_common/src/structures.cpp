#include <fstream>
#include <yaml-cpp/yaml.h>

#include <bwi_planning_common/structures.h>

namespace bwi_planning_common {
  
  void readLocationFile(const std::string& filename, 
      std::vector<std::string>& locations, std::vector<int32_t>& location_map) {
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    locations.clear();
    const YAML::Node &loc_node = doc["locations"];
    for (size_t i = 0; i < loc_node.size(); i++) {
      std::string location;
      loc_node[i] >> location;
      locations.push_back(location);
    }
    const YAML::Node &data_node = doc["data"];
    location_map.resize(data_node.size());
    for (size_t i = 0; i < data_node.size(); i++) {
      data_node[i] >> location_map[i];

    }
  }

  void readDoorFile(const std::string& filename, std::vector<Door>& doors) {
    std::ifstream fin(filename.c_str());
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    doors.clear();
    for (size_t i = 0; i < doc.size(); i++) {
      Door door;
      const YAML::Node &approach_node = doc[i]["approach"];
      for (size_t j = 0; j < 2; ++j) {
        approach_node[j]["from"] >> door.approach_names[j];
        approach_node[j]["point"][0] >> door.approach_points[j].x; 
        approach_node[j]["point"][1] >> door.approach_points[j].y; 
        approach_node[j]["point"][2] >> door.approach_yaw[j]; 
      }
      doc[i]["name"] >> door.name;
      try {
        doc[i]["width"] >> door.width;
      } catch(YAML::TypedKeyNotFound<std::string>& e) {
        door.width = 0.5;
      }
      doors.push_back(door);
    }
  }

  size_t resolveDoor(const std::string& door, const std::vector<Door>& doors) {
    
    for (size_t i = 0; i < doors.size(); ++i) {
      if (doors[i].name == door) {
        return i;
      }
    }

    return bwi_planning_common::NO_DOOR_IDX;
  }

} /* bwi_common */

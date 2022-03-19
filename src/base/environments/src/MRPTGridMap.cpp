//
// Created by ksatyaki on 3/18/22.
//

#include "base/environments/MRPTGridMap.h"
MRPTGridMap::MRPTGridMap(const std::string& yaml_file_name) {
  YAML::Node map_yaml_node = YAML::LoadFile(yaml_file_name);


  std::cout << map_yaml_node["image"].as<std::string>() << "\n"
            << "Origin x: " << map_yaml_node["origin"][0] << "\n"
            << "Origin y: " << map_yaml_node["origin"][1];
}

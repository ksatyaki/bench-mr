//
// Created by ksatyaki on 3/18/22.
//

#include "base/environments/MRPTGridMap.h"
MRPTGridMap::MRPTGridMap(const std::string& yaml_file_name) {
  YAML::Node map_yaml_node = YAML::LoadFile(yaml_file_name);

  this->mrpt_map_ = std::make_shared<mrpt::maps::COccupancyGridMap2D>();
  auto folder = yaml_file_name.substr(0, yaml_file_name.find_last_of("/"));

  auto origin_x = map_yaml_node["origin"][0].as<float>();
  auto origin_y = map_yaml_node["origin"][1].as<float>();
  auto resolution = map_yaml_node["resolution"].as<float>();
  auto full_image_path =
      folder + "/" + map_yaml_node["image"].as<std::string>();
  if (this->mrpt_map_->loadFromBitmapFile(
          full_image_path, resolution,
          mrpt::math::TPoint2D(-origin_x / resolution,
                               -origin_y / resolution))) {
    printf("Successfully loaded an MRPT Gridmap from ros-style YAML.\n");
  }

  std::cout << "X min: " << mrpt_map_->getXMin() << ", "
            << "X max: " << mrpt_map_->getXMax() << ", "
            << "Y min: " << mrpt_map_->getYMin() << ", "
            << "Y max: " << mrpt_map_->getYMax() << std::endl;

  this->_bounds.setLow(0, mrpt_map_->getXMin());
  this->_bounds.setHigh(0, mrpt_map_->getXMax());

  this->_bounds.setLow(1, mrpt_map_->getYMin());
  this->_bounds.setHigh(1, mrpt_map_->getYMax());

  this->_threshold = map_yaml_node["occupied_thresh"].as<double>();
  this->_file = yaml_file_name;
}

double MRPTGridMap::distance(double x, double y) {
  return mrpt_map_->computeClearance(x, y, 10.0);
}

bool MRPTGridMap::collides(double x, double y) {
  // For some reason the values are (1 - <>) in the MRPT map
  // return (1.0 - mrpt_map_->getPos(x, y)) > this->_threshold;
  return mrpt_map_->computeClearance(x, y, 0.3) < 0.3;
}

bool MRPTGridMap::collides(const Polygon& polygon) {
  bool collision = false;
  for (auto point : polygon.points) {
    if (collides(point.x, point.y)) {
      collision = true;
      break;
    }
  }
  return collision;
}

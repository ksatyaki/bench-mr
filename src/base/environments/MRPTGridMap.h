//
// Created by ksatyaki on 3/18/22.
//

#pragma once

#include <base/Environment.h>
#include <yaml-cpp/yaml.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <memory>

class MRPTGridMap : public Environment {
  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> mrpt_map_;

  float _threshold{0.0};
  std::string _file;
 public:
  MRPTGridMap() = default;
  explicit MRPTGridMap(const std::string& yaml_file_name);
  ~MRPTGridMap() override = default;

  virtual bool collides(double x, double y) override;
  virtual bool collides(const Polygon &polygon) override;
  virtual double distance(double x, double y) override;
  virtual std::string name() const override { return "MRPT Grid-map"; }

  virtual void to_json(nlohmann::json &j) override {
    j["type"] = "yaml";
    j["file"] = _file;
    j["threshold"] = _threshold;
    j["min_x"] = mrpt_map_->getXMin();
    j["max_x"] = mrpt_map_->getXMax();
    j["min_y"] = mrpt_map_->getYMin();
    j["max_y"] = mrpt_map_->getYMax();
    j["start"] = start();
    j["goal"] = goal();
    j["name"] = name();
  }
};


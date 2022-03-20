//
// Created by ksatyaki on 3/18/22.
//

#pragma once

#include <base/Environment.h>
#include <yaml-cpp/yaml.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <memory>

class MRPTGridMap : Environment {
  std::shared_ptr<mrpt::maps::COccupancyGridMap2D> mrpt_map_;

  float _thresdhold{0.0};
 public:
  MRPTGridMap() = default;
  MRPTGridMap(const std::string& yaml_file_name);

  virtual bool collides(double x, double y) override;
  virtual bool collides(const Polygon &polygon) override;
  virtual double distance(double x, double y) override;
  virtual std::string name() const override { return "MRPT Grid-map"; }

  virtual void to_json(nlohmann::json &j) override {
    j["type"] = "mrpt";
    j["width"] = width();
    j["height"] = height();
    j["start"] = start();
    j["goal"] = goal();
    j["name"] = name();
  }
};


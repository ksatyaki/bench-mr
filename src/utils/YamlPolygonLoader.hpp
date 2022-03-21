#pragma once

#include <yaml-cpp/yaml.h>

#include "base/Primitives.h"

/**
 * This class loads polygons from an SVG file. Polygons must be stored as path
 * tags and only the Inkscape-style formatting is supported at the moment.
 * For examples of the supported SVG files, see files in `bin/parking`.
 */
class YamlPolygonLoader {
 public:
  static std::vector<Polygon> load(const std::string &filename) {
    YAML::Node polygon_yaml = YAML::LoadFile(filename);
    Polygon p;
    for (size_t i = 0; i < polygon_yaml["points"].size(); i++) {
      p.points.push_back(Point(polygon_yaml["points"][i][0].as<double>(),
                               polygon_yaml["points"][i][1].as<double>()));
    }
    return {p};
  }
};

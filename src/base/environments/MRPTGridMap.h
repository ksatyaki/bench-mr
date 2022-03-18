//
// Created by ksatyaki on 3/18/22.
//

#pragma once

#include <base/Environment.h>
#include <yaml.h>

class MRPTGridMap : Environment {
  MRPTGridMap() = default;
  MRPTGridMap(const std::string& yaml_file_name);
};


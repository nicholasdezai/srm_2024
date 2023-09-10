#include "srm/common/config.h"

#include <yaml-cpp/yaml.h>

namespace srm {

Config& Config::Instance() {
  static Config config;
  return config;
}

bool Config::Parse(std::string config_file) {
  std::function<bool(YAML::Node, std::string)> dfs = [&](YAML::Node REF_IN u, std::string name) {
    if (u.IsMap() && u.Tag() != "!opencv-matrix") {
      for (const auto& v : u)
        if (!dfs(v.second, name + "." + v.first.as<std::string>())) return false;
    } else {
      if (registry_.count(name)) return true;
      if (u.IsScalar()) {
        try {
          registry_[name] = u.as<int>();
        } catch (const YAML::BadConversion& e) {
          try {
            registry_[name] = u.as<double>();
          } catch (const YAML::BadConversion& e) {
            try {
              registry_[name] = u.as<bool>();
            } catch (const YAML::BadConversion& e) {
              try {
                registry_[name] = u.as<std::string>();
              } catch (const YAML::BadConversion& e) {
                LOG(ERROR) << "Unknow type for " << name;
              }
            }
          }
        }
      } else if (u.IsMap()) {
        if (u.Tag() != "!opencv-matrix" || !u["rows"] || !u["cols"] || !u["data"]) {
          LOG(INFO) << "The format of Matrix " << name << " is wrong.";
          return false;
        }
        int rows = u["rows"].as<int>();
        int cols = u["cols"].as<int>();
        cv::Mat mat(rows, cols, CV_64F);
        YAML::Node data_node = u["data"];
        for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
            mat.at<double>(i, j) = data_node[i * cols + j].as<double>();
          }
        }
        registry_[name] = std::move(mat);
      } else if (u.IsSequence()) {
        std::vector<double> vec;
        for (auto REF_IN it : u) vec.push_back(it.as<double>());
        registry_[name] = std::move(vec);
      }
    }
    return true;
  };
  auto nodes = YAML::LoadAllFromFile("../config.yaml");
  for (auto node : nodes) {
    for (auto it : node) {
      if (!dfs(it.second, it.first.as<std::string>())) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace srm

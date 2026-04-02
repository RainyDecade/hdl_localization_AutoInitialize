#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

#include "btc_init_localizer/BTC.h"

namespace btc_init_localizer {

struct DatabaseEntry {
  int index = -1;
  std::string pcd_path;
  std::string plane_path;
  std::string desc_path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  std::vector<STD> stds;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud;
};

bool saveDatabaseEntry(const std::string& root_dir, const DatabaseEntry& entry);

bool loadDatabase(const std::string& root_dir,
                  std::vector<DatabaseEntry>* entries,
                  std::string* error_msg);

}  // namespace btc_init_localizer

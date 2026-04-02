#include "btc_init_localizer/EstimateInitialPose.h"
#include "btc_init_localizer/BTC.h"
#include "btc_init_localizer/btc_database.hpp"

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bil = btc_init_localizer;

class BTCNDTInitializer {
public:
  BTCNDTInitializer() : nh_(), pnh_("~") {
    pnh_.param<std::string>("db_dir", db_dir_, "/tmp/btc_db");
    pnh_.param<std::string>("lidar_topic", lidar_topic_, "/velodyne_points");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("initialpose_topic", initialpose_topic_, "/initialpose");
    pnh_.param<double>("service_wait_cloud_sec", wait_cloud_sec_, 1.0);
    pnh_.param<double>("btc_score_threshold", btc_score_threshold_, 0.15);
    pnh_.param<int>("is_high_fly", is_high_fly_, 0);

    pnh_.param<double>("ndt_resolution", ndt_resolution_, 1.0);
    pnh_.param<double>("ndt_step_size", ndt_step_size_, 0.1);
    pnh_.param<double>("ndt_trans_eps", ndt_trans_eps_, 0.01);
    pnh_.param<int>("ndt_max_iter", ndt_max_iter_, 40);
    pnh_.param<int>("ndt_num_threads", ndt_num_threads_, 4);
    pnh_.param<int>("ndt_neighborhood", ndt_neighborhood_, 2);

    read_parameters(pnh_, config_setting_, is_high_fly_);
    pnh_.param<int>("skip_near_num", config_setting_.skip_near_num_, -1);
    config_setting_.icp_threshold_ = static_cast<float>(btc_score_threshold_);

    std::vector<bil::DatabaseEntry> loaded_entries;
    std::string error;
    if (!bil::loadDatabase(db_dir_, &loaded_entries, &error)) {
      ROS_FATAL_STREAM("Failed to load BTC database: " << error << " db_dir=" << db_dir_);
      ros::shutdown();
      return;
    }

    desc_manager_.reset(new STDescManager(config_setting_));

    for (auto& entry : loaded_entries) {
      if (entry.stds.empty() || !entry.plane_cloud || entry.plane_cloud->empty()) {
        continue;
      }

      const int local_index = static_cast<int>(entries_.size());
      entry.index = local_index;
      for (auto& std_item : entry.stds) {
        std_item.frame_number_ = local_index;
      }

      desc_manager_->plane_cloud_vec_.push_back(entry.plane_cloud);
      desc_manager_->AddSTDescs(entry.stds);
      entries_.push_back(entry);
    }

    if (entries_.empty()) {
      ROS_FATAL_STREAM("BTC database is empty after filtering. db_dir=" << db_dir_);
      ros::shutdown();
      return;
    }

    initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic_, 1, true);
    service_ = nh_.advertiseService("btc_initialize", &BTCNDTInitializer::serviceCallback, this);

    ROS_INFO_STREAM("btc_ndt_initializer ready: entries=" << entries_.size() << " service=/btc_initialize");
  }

private:
  bool serviceCallback(btc_init_localizer::EstimateInitialPose::Request& req,
                       btc_init_localizer::EstimateInitialPose::Response& res) {
    (void)req;

    const auto cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        lidar_topic_, nh_, ros::Duration(wait_cloud_sec_));
    if (!cloud_msg) {
      res.success = false;
      res.message = "timeout waiting lidar topic";
      return true;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *source_cloud);
    if (source_cloud->empty()) {
      res.success = false;
      res.message = "empty lidar cloud";
      return true;
    }

    std::vector<STD> query_stds;
    desc_manager_->GenerateSTDescs(source_cloud, query_stds, static_cast<int>(desc_manager_->current_frame_id_));

    if (query_stds.empty() || desc_manager_->plane_cloud_vec_.empty() ||
        !desc_manager_->plane_cloud_vec_.back() || desc_manager_->plane_cloud_vec_.back()->empty()) {
      if (!desc_manager_->plane_cloud_vec_.empty()) {
        desc_manager_->plane_cloud_vec_.pop_back();
      }
      res.success = false;
      res.message = "failed to generate BTC descriptors";
      return true;
    }

    auto query_plane_cloud = desc_manager_->plane_cloud_vec_.back();

    std::pair<int, double> loop_result(-1, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    std::vector<std::pair<STD, STD>> loop_std_pair;
    desc_manager_->SearchLoop(query_stds, loop_result, loop_transform, loop_std_pair, query_plane_cloud);

    // Remove transient current-frame plane cloud, keep offline DB planes intact.
    desc_manager_->plane_cloud_vec_.pop_back();

    if (loop_result.first < 0 || loop_result.first >= static_cast<int>(entries_.size())) {
      res.success = false;
      res.message = "BTC matching failed";
      res.btc_score = static_cast<float>(loop_result.second);
      res.matched_index = -1;
      return true;
    }

    if (loop_result.second < btc_score_threshold_) {
      res.success = false;
      res.message = "BTC score below threshold";
      res.btc_score = static_cast<float>(loop_result.second);
      res.matched_index = entries_[loop_result.first].index;
      return true;
    }

    const auto& matched = entries_[loop_result.first];

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::string target_path = matched.pcd_path;
    if (!boost::filesystem::path(target_path).is_absolute()) {
      target_path = (boost::filesystem::path(db_dir_) / target_path).string();
    }
    if (pcl::io::loadPCDFile(target_path, *target_cloud) != 0 || target_cloud->empty()) {
      res.success = false;
      res.message = "failed to load matched target pcd";
      res.btc_score = static_cast<float>(loop_result.second);
      res.matched_index = matched.index;
      return true;
    }

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3, 3>(0, 0) = loop_transform.second.cast<float>();
    init_guess(0, 3) = static_cast<float>(loop_transform.first.x());
    init_guess(1, 3) = static_cast<float>(loop_transform.first.y());
    init_guess(2, 3) = static_cast<float>(loop_transform.first.z());

    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_trans_eps_);
    ndt.setMaximumIterations(ndt_max_iter_);
    ndt.setNumThreads(ndt_num_threads_);

    if (ndt_neighborhood_ == 0) {
      ndt.setNeighborhoodSearchMethod(pclomp::KDTREE);
    } else if (ndt_neighborhood_ == 1) {
      ndt.setNeighborhoodSearchMethod(pclomp::DIRECT26);
    } else if (ndt_neighborhood_ == 3) {
      ndt.setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else {
      ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    }

    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(source_cloud);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    ndt.align(aligned, init_guess);

    if (!ndt.hasConverged()) {
      res.success = false;
      res.message = "NDT did not converge";
      res.btc_score = static_cast<float>(loop_result.second);
      res.matched_index = matched.index;
      return true;
    }

    const Eigen::Matrix4f tf_historical_current = ndt.getFinalTransformation();
    const Eigen::Matrix4f tf_map_historical = composePoseMatrix(matched.position, matched.orientation);
    const Eigen::Matrix4f tf_map_current = tf_map_historical * tf_historical_current;

    Eigen::Quaternionf q(tf_map_current.block<3, 3>(0, 0));
    q.normalize();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose.position.x = tf_map_current(0, 3);
    pose_msg.pose.pose.position.y = tf_map_current(1, 3);
    pose_msg.pose.pose.position.z = tf_map_current(2, 3);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    for (double& c : pose_msg.pose.covariance) {
      c = 0.0;
    }
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.2;

    initialpose_pub_.publish(pose_msg);

    res.success = true;
    res.message = "success";
    res.initial_pose = pose_msg;
    res.btc_score = static_cast<float>(loop_result.second);
    res.matched_index = matched.index;

    ROS_INFO_STREAM("Init pose published. idx=" << matched.index
                    << " btc_score=" << loop_result.second
                    << " ndt_fitness=" << ndt.getFitnessScore());
    return true;
  }

private:
  static Eigen::Matrix4f composePoseMatrix(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf qf(static_cast<float>(q.w()),
                          static_cast<float>(q.x()),
                          static_cast<float>(q.y()),
                          static_cast<float>(q.z()));
    qf.normalize();
    m.block<3, 3>(0, 0) = qf.toRotationMatrix();
    m(0, 3) = static_cast<float>(t.x());
    m(1, 3) = static_cast<float>(t.y());
    m(2, 3) = static_cast<float>(t.z());
    return m;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher initialpose_pub_;
  ros::ServiceServer service_;

  std::unique_ptr<STDescManager> desc_manager_;
  std::vector<bil::DatabaseEntry> entries_;

  std::string db_dir_;
  std::string lidar_topic_;
  std::string map_frame_;
  std::string initialpose_topic_;

  double wait_cloud_sec_ = 1.0;
  double btc_score_threshold_ = 0.15;
  int is_high_fly_ = 0;

  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_trans_eps_ = 0.01;
  int ndt_max_iter_ = 40;
  int ndt_num_threads_ = 4;
  int ndt_neighborhood_ = 2;

  ConfigSetting config_setting_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "btc_ndt_initializer_node");
  BTCNDTInitializer node;
  ros::spin();
  return 0;
}

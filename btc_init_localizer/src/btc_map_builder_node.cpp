#include "btc_init_localizer/BTC.h"
#include "btc_init_localizer/btc_database.hpp"

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

namespace bil = btc_init_localizer;

class BTCMapBuilder {
public:
  BTCMapBuilder() : nh_(), pnh_("~") {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/velodyne_points");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("output_dir", output_dir_, "/tmp/btc_db");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<int>("save_every_n", save_every_n_, 3);
    pnh_.param<int>("sync_queue_size", sync_queue_size_, 20);
    pnh_.param<int>("is_high_fly", is_high_fly_, 0);

    read_parameters(pnh_, config_setting_, is_high_fly_);

    boost::filesystem::create_directories(boost::filesystem::path(output_dir_) / "clouds");
    boost::filesystem::create_directories(boost::filesystem::path(output_dir_) / "planes");
    boost::filesystem::create_directories(boost::filesystem::path(output_dir_) / "entries");

    saved_count_ = countExistingEntries();

    cloud_sub_.subscribe(nh_, cloud_topic_, 20);
    odom_sub_.subscribe(nh_, odom_topic_, 200);
    sync_.reset(new Sync(SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_));
    sync_->registerCallback(boost::bind(&BTCMapBuilder::syncCallback, this, _1, _2));

    ROS_INFO_STREAM("btc_map_builder ready. cloud_topic=" << cloud_topic_
                    << " odom_topic=" << odom_topic_
                    << " output_dir=" << output_dir_);
  }

private:
  int countExistingEntries() const {
    const std::string index_path = (boost::filesystem::path(output_dir_) / "index.txt").string();
    std::ifstream ifs(index_path);
    if (!ifs.is_open()) {
      return 0;
    }

    int count = 0;
    std::string line;
    while (std::getline(ifs, line)) {
      if (!line.empty()) {
        ++count;
      }
    }
    return count;
  }

  void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const nav_msgs::OdometryConstPtr& odom_msg) {
    ++recv_count_;
    if (save_every_n_ > 1 && (recv_count_ % save_every_n_) != 0) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty()) {
      return;
    }

    STDescManager desc_manager(config_setting_);
    desc_manager.current_frame_id_ = static_cast<unsigned int>(saved_count_);

    std::vector<STD> stds;
    desc_manager.GenerateSTDescs(cloud, stds, saved_count_);
    if (stds.empty() || desc_manager.plane_cloud_vec_.empty() || !desc_manager.plane_cloud_vec_.back() ||
        desc_manager.plane_cloud_vec_.back()->empty()) {
      ROS_WARN_STREAM_THROTTLE(1.0, "Skip frame due to empty BTC descriptors or empty plane cloud");
      return;
    }

    bil::DatabaseEntry entry;
    entry.index = saved_count_;
    entry.position = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
    entry.orientation = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                           odom_msg->pose.pose.orientation.x,
                                           odom_msg->pose.pose.orientation.y,
                                           odom_msg->pose.pose.orientation.z);
    entry.orientation.normalize();

    if (!map_frame_.empty() && !odom_msg->header.frame_id.empty() && odom_msg->header.frame_id != map_frame_) {
      ROS_WARN_STREAM_THROTTLE(2.0, "Odometry frame_id is " << odom_msg->header.frame_id
                               << ", expected " << map_frame_);
    }

    std::ostringstream pcd_name;
    pcd_name << "clouds/" << std::setw(6) << std::setfill('0') << entry.index << ".pcd";
    entry.pcd_path = pcd_name.str();

    std::ostringstream plane_name;
    plane_name << "planes/" << std::setw(6) << std::setfill('0') << entry.index << ".pcd";
    entry.plane_path = plane_name.str();

    entry.stds = stds;
    for (auto& std_item : entry.stds) {
      std_item.frame_number_ = entry.index;
    }

    const std::string abs_pcd = (boost::filesystem::path(output_dir_) / entry.pcd_path).string();
    if (pcl::io::savePCDFileBinaryCompressed(abs_pcd, *cloud) != 0) {
      ROS_ERROR_STREAM("Failed to save pcd: " << abs_pcd);
      return;
    }

    const std::string abs_plane = (boost::filesystem::path(output_dir_) / entry.plane_path).string();
    if (pcl::io::savePCDFileBinaryCompressed(abs_plane, *desc_manager.plane_cloud_vec_.back()) != 0) {
      ROS_ERROR_STREAM("Failed to save plane cloud: " << abs_plane);
      return;
    }

    if (!bil::saveDatabaseEntry(output_dir_, entry)) {
      ROS_ERROR("Failed to save BTC database entry");
      return;
    }

    ++saved_count_;
    ROS_INFO_STREAM_THROTTLE(0.5, "Saved BTC entry count=" << saved_count_ << " std_count=" << entry.stds.size());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<Sync> sync_;

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string output_dir_;
  std::string map_frame_;

  int save_every_n_ = 3;
  int sync_queue_size_ = 20;
  int is_high_fly_ = 0;
  int recv_count_ = 0;
  int saved_count_ = 0;

  ConfigSetting config_setting_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "btc_map_builder_node");
  BTCMapBuilder node;
  ros::spin();
  return 0;
}

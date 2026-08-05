#ifndef LIVOX_LASER_SIMULATION_LIVOX_LIDAR_SYSTEM_HH_
#define LIVOX_LASER_SIMULATION_LIVOX_LIDAR_SYSTEM_HH_

#include <gz/math/Pose3.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <fast_livo/msg/custom_msg.hpp>

#include "livox_laser_simulation/livox_depth_sampler.hh"
#include "livox_laser_simulation/msg/custom_msg.hpp"

namespace livox_laser_simulation
{

enum class PointCloudType
{
  SENSOR_MSG_POINT_CLOUD2_POINTXYZ = 1,
  SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT = 2,
  LIVOX_LASER_SIMULATION_CUSTOM_MSG = 3,
  FAST_LIVO_CUSTOM_MSG = 4,
};

class LivoxLidarSystem : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
public:
  LivoxLidarSystem() = default;
  ~LivoxLidarSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_event_mgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void InitializeROS();
  void OnGpuScan(const gz::msgs::LaserScan &_scan);
  void PublishPointCloud(const std::vector<LivoxRayResult> &_results,
                         const rclcpp::Time &_stamp);
  void PublishPointCloud2(const std::vector<LivoxRayResult> &_results,
                          const rclcpp::Time &_stamp, bool _livox_fields);
  void PublishLivoxCustomMsg(const std::vector<LivoxRayResult> &_results,
                             const rclcpp::Time &_stamp);
  void PublishFastLivoCustomMsg(const std::vector<LivoxRayResult> &_results,
                                const rclcpp::Time &_stamp);
  void BroadcastTF(const rclcpp::Time &_stamp,
                   const gz::math::Pose3d &_parent_pose,
                   const gz::math::Pose3d &_sensor_pose);

  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity parent_entity_{gz::sim::kNullEntity};
  std::string sensor_name_;
  std::string parent_name_;
  gz::math::Pose3d sensor_pose_;
  gz::math::Pose3d parent_pose_;

  std::unique_ptr<LivoxDepthSampler> depth_sampler_;
  std::string csv_file_name_;
  std::string gpu_topic_{"/livox/lidar/scan"};
  std::string ros_topic_{"/livox/lidar"};
  std::string frame_name_;
  int sample_step_{20000};
  int downsample_{1};
  int line_count_{4};
  size_t startup_skip_scans_{0};
  size_t startup_scans_seen_{0};
  PointCloudType publish_pointcloud_type_{
    PointCloudType::SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT};
  double min_dist_{0.1};
  double max_dist_{40.0};
  double point_rate_{200000.0};
  bool use_inf_{false};
  bool publish_point_time_offsets_{true};

  size_t curr_start_index_{0};
  std::chrono::steady_clock::duration sim_time_{};
  bool configured_{false};
  bool grid_error_reported_{false};

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<livox_laser_simulation::msg::CustomMsg>::SharedPtr custom_msg_pub_;
  rclcpp::Publisher<fast_livo::msg::CustomMsg>::SharedPtr fast_livo_custom_msg_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
  // Destroy the subscriber before state used by its callback.
  gz::transport::Node transport_node_;
};

}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION_LIVOX_LIDAR_SYSTEM_HH_

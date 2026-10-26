#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/joining_thread.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

using DisplayPoint = pcl::PointXYZI;

class RvizViewer : public ExtensionModule {
public:
  RvizViewer();
  ~RvizViewer() override;
  void stop() override;

private:
  struct QueueItem {
    enum class Kind { NEW_FRAME, ODOMETRY_IMU };
    Kind kind;
    KeyFrame::ConstPtr frame{nullptr};
  };

  void setup_callbacks();

  void on_new_frame(const KeyFrame::ConstPtr& frame);
  void on_odometry_imu(const KeyFrame::ConstPtr& frame);

  void enqueue(QueueItem::Kind kind, const KeyFrame::ConstPtr& frame);
  void push_item(const QueueItem& item);

  void processing_thread();
  void process_new_frame(const KeyFrame::ConstPtr& frame);
  void process_odometry_imu(const KeyFrame::ConstPtr& frame);

  void publish_odometry(const KeyFrame::ConstPtr& frame, ros::Publisher& publisher);
  void publish_pointcloud(
    double stamp,
    const std::string& frame_id,
    const pcl::PointCloud<DisplayPoint>::ConstPtr& cloud,
    ros::Publisher& publisher);

  void publish_tf(const KeyFrame::ConstPtr& frame);
  void prepare_clouds(
    const KeyFrame::ConstPtr& frame,
    pcl::PointCloud<DisplayPoint>::Ptr cloud_imu,
    pcl::PointCloud<DisplayPoint>::Ptr cloud_world);

  CallbackSubscription<void(const KeyFrame::ConstPtr&)> on_new_frame_sub;
  CallbackSubscription<void(const KeyFrame::ConstPtr&)> on_odometry_imu_sub;

  ros::NodeHandle nh{"~"};
  ros::Publisher odometry_pub;
  ros::Publisher odometry_imu_pub;
  ros::Publisher cloud_imu_pub;
  ros::Publisher cloud_world_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster{nullptr};

  static constexpr std::size_t queue_capacity = 4096;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<QueueItem> queue{queue_capacity};
  std::atomic<bool> kill_switch{false};
  JoiningThread processing_thread_obj;
  std::size_t dropped_items{0};

  std::string world_frame_id{"world"};
  std::string imu_frame_id{"imu"};
  bool enable_odometry_publish{true};
  bool enable_odometry_imu_publish{true};
  bool enable_cloud_imu_publish{true};
  bool enable_cloud_world_publish{true};
  bool enable_tf_publish{true};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <boost/circular_buffer.hpp>

#include <asuka/core/callbacks.hpp>
#include <asuka/modules/freedom/freedom_callbacks.hpp>
#include <asuka/modules/freedom/freedom_types.hpp>
#include <asuka/utility/extension_module.hpp>
#include <asuka/utility/joining_thread.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {

// ROS publisher for the freedom extension's result streams (online dynamic
// object removal). Subscribes the on_freedom_* callback slots and turns the
// payloads into /asuka/freedom/* topics: point clouds, depth images and the
// range-box markers. The slot callbacks only enqueue -- the freedom module
// emits from its processing thread, and observers must not block. The
// serialization and publishing run on this module's own processing thread.
//
// Publishing gates mirror the payload contract: freedom leaves a payload
// field null when the corresponding stream is disabled in its config
// ("publish" block), and a null field is simply not published here.
// All geometry is world-frame (the payloads carry world coordinates); the
// frame is configurable via the "freedom_viewer" block of config_extensions.
class FreedomViewer : public ExtensionModule {
public:
  FreedomViewer();
  ~FreedomViewer() override;
  void stop() override;

private:
  struct QueueItem {
    enum class Kind { SCAN_REMOVAL, RAYCAST_ENHANCEMENT, MAP_REMOVAL };
    Kind kind;
    FreedomScanRemoval::ConstPtr scan_removal{nullptr};
    FreedomRaycastEnhancement::ConstPtr raycast_enhancement{nullptr};
    FreedomMapRemoval::ConstPtr map_removal{nullptr};
  };

  // Slot callbacks: enqueue only, never publish inline.
  void on_scan_removal(const FreedomScanRemoval::ConstPtr& payload);
  void on_raycast_enhancement(const FreedomRaycastEnhancement::ConstPtr& payload);
  void on_map_removal(const FreedomMapRemoval::ConstPtr& payload);

  void enqueue(QueueItem::Kind kind, const FreedomScanRemoval::ConstPtr& scan_removal,
               const FreedomRaycastEnhancement::ConstPtr& raycast_enhancement,
               const FreedomMapRemoval::ConstPtr& map_removal);

  // The publishing work runs exclusively on the processing thread so the
  // freedom module's emission thread never pays for it.
  void processing_thread();

  template <typename PointT>
  void publish_cloud(double stamp, const pcl::PointCloud<PointT>& cloud, ros::Publisher& publisher);
  void publish_image(double stamp, const cv::Mat& image, ros::Publisher& publisher);
  void publish_range_marker(
    double stamp,
    const FreedomBox& box,
    const std::string& ns,
    float r,
    float g,
    float b,
    ros::Publisher& publisher);

  void process_scan_removal(const FreedomScanRemoval::ConstPtr& payload);
  void process_raycast_enhancement(const FreedomRaycastEnhancement::ConstPtr& payload);
  void process_map_removal(const FreedomMapRemoval::ConstPtr& payload);

  // RAII subscriptions: destruction removes the callbacks. Declared before
  // the processing thread so destruction joins the thread first, then
  // unsubscribes.
  CallbackSubscription<void(const FreedomScanRemoval::ConstPtr&)> scan_removal_sub;
  CallbackSubscription<void(const FreedomRaycastEnhancement::ConstPtr&)> raycast_enhancement_sub;
  CallbackSubscription<void(const FreedomMapRemoval::ConstPtr&)> map_removal_sub;

  ros::NodeHandle nh{"~"};
  ros::Publisher scan_blocks_pub;
  ros::Publisher scan_voxels_pub;
  ros::Publisher clusters_pub;
  ros::Publisher depth_image_pub;
  ros::Publisher enhanced_depth_image_pub;
  ros::Publisher enhanced_pointcloud_pub;
  ros::Publisher raycasted_blocks_pub;
  ros::Publisher raycasted_voxels_pub;
  ros::Publisher free_blocks_pub;
  ros::Publisher free_voxels_pub;
  ros::Publisher static_blocks_pub;
  ros::Publisher static_voxels_pub;
  ros::Publisher static_subvoxels_pub;
  ros::Publisher static_pointcloud_pub;
  ros::Publisher scan_map_range_pub;
  ros::Publisher raycast_range_pub;
  ros::Publisher local_map_range_pub;

  static constexpr std::size_t queue_capacity = 512;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  boost::circular_buffer<QueueItem> queue{queue_capacity};
  std::atomic<bool> kill_switch{false};
  JoiningThread processing_thread_obj;
  std::size_t dropped_items{0};

  std::string world_frame_id{"camera_init"};

  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace asuka

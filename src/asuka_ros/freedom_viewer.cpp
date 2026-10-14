#include <asuka_ros/freedom_viewer.hpp>

#include <sensor_msgs/image_encodings.h>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <asuka/core/callbacks.hpp>
#include <asuka/utility/config.hpp>

namespace asuka {

FreedomViewer::FreedomViewer() {
  logger = create_module_logger("freedom_viz");

  const Config config = load_config("config_extensions");
  world_frame_id = config.param<std::string>("freedom_viewer", "world_frame_id", "camera_init");
  const std::string prefix = config.param<std::string>("freedom_viewer", "topic_prefix", "/asuka/freedom");

  scan_blocks_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/scan_blocks", 10);
  scan_voxels_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/scan_voxels", 10);
  clusters_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/clusters", 10);
  depth_image_pub = nh.advertise<sensor_msgs::Image>(prefix + "/depth_image", 10);
  enhanced_depth_image_pub = nh.advertise<sensor_msgs::Image>(prefix + "/enhanced_depth_image", 10);
  enhanced_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/enhanced_pointcloud", 10);
  raycasted_blocks_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/raycasted_blocks", 10);
  raycasted_voxels_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/raycasted_voxels", 10);
  free_blocks_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/free_blocks", 10);
  free_voxels_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/free_voxels", 10);
  static_blocks_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/static_blocks", 10);
  static_voxels_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/static_voxels", 10);
  static_subvoxels_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/static_subvoxels", 10);
  static_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/static_pointcloud", 10);
  scan_map_range_pub = nh.advertise<visualization_msgs::Marker>(prefix + "/scan_map_range", 10);
  raycast_range_pub = nh.advertise<visualization_msgs::Marker>(prefix + "/raycast_range", 10);
  local_map_range_pub = nh.advertise<visualization_msgs::Marker>(prefix + "/local_map_range", 10);

  scan_removal_sub = FreedomCallbacks::on_freedom_scan_removal.subscribe(
    std::bind(&FreedomViewer::on_scan_removal, this, std::placeholders::_1));
  raycast_enhancement_sub = FreedomCallbacks::on_freedom_raycast_enhancement.subscribe(
    std::bind(&FreedomViewer::on_raycast_enhancement, this, std::placeholders::_1));
  map_removal_sub = FreedomCallbacks::on_freedom_map_removal.subscribe(
    std::bind(&FreedomViewer::on_map_removal, this, std::placeholders::_1));

  kill_switch = false;
  processing_thread_obj = JoiningThread(&FreedomViewer::processing_thread, this);

  logger->info("freedom_viewer initialized (frame_id={}, prefix={})", world_frame_id, prefix);
}

FreedomViewer::~FreedomViewer() {
  stop();
}

void FreedomViewer::stop() {
  if (!kill_switch.exchange(true)) {
    queue_cv.notify_all();
  }
  processing_thread_obj.join();
}

void FreedomViewer::on_scan_removal(const FreedomScanRemoval::ConstPtr& payload) {
  if (payload) enqueue(QueueItem::Kind::SCAN_REMOVAL, payload, nullptr, nullptr);
}

void FreedomViewer::on_raycast_enhancement(const FreedomRaycastEnhancement::ConstPtr& payload) {
  if (payload) enqueue(QueueItem::Kind::RAYCAST_ENHANCEMENT, nullptr, payload, nullptr);
}

void FreedomViewer::on_map_removal(const FreedomMapRemoval::ConstPtr& payload) {
  if (payload) enqueue(QueueItem::Kind::MAP_REMOVAL, nullptr, nullptr, payload);
}

void FreedomViewer::enqueue(QueueItem::Kind kind, const FreedomScanRemoval::ConstPtr& scan_removal,
                            const FreedomRaycastEnhancement::ConstPtr& raycast_enhancement,
                            const FreedomMapRemoval::ConstPtr& map_removal) {
  QueueItem item;
  item.kind = kind;
  item.scan_removal = scan_removal;
  item.raycast_enhancement = raycast_enhancement;
  item.map_removal = map_removal;
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (queue.full()) {
      if (++dropped_items % 100 == 1) {
        logger->warn("freedom_viewer queue overflow, dropping oldest items ({} dropped)", dropped_items);
      }
    }
    queue.push_back(item);
  }
  queue_cv.notify_one();
}

void FreedomViewer::processing_thread() {
  while (true) {
    QueueItem item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_cv.wait(lock, [this] { return !queue.empty() || kill_switch; });
      if (kill_switch && queue.empty()) return;
      item = queue.front();
      queue.pop_front();
    }

    switch (item.kind) {
      case QueueItem::Kind::SCAN_REMOVAL: process_scan_removal(item.scan_removal); break;
      case QueueItem::Kind::RAYCAST_ENHANCEMENT:
        process_raycast_enhancement(item.raycast_enhancement);
        break;
      case QueueItem::Kind::MAP_REMOVAL: process_map_removal(item.map_removal); break;
    }
  }
}

template <typename PointT>
void FreedomViewer::publish_cloud(double stamp, const pcl::PointCloud<PointT>& cloud, ros::Publisher& publisher) {
  if (cloud.empty()) return;
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = world_frame_id;
  publisher.publish(msg);
}

// Only the payload's two cloud flavors exist.
template void FreedomViewer::publish_cloud<pcl::PointXYZ>(double, const pcl::PointCloud<pcl::PointXYZ>&, ros::Publisher&);
template void FreedomViewer::publish_cloud<pcl::PointXYZRGB>(double, const pcl::PointCloud<pcl::PointXYZRGB>&,
                                                             ros::Publisher&);

void FreedomViewer::publish_image(double stamp, const cv::Mat& image, ros::Publisher& publisher) {
  if (image.empty()) return;
  // Single materialization at the ROS boundary: the payload's Mat is shared
  // and never mutated; the message takes one continuous copy. (Hand-rolled
  // instead of cv_bridge: the payload is already bgr8 and this cv_bridge
  // build has no cv::Mat overload.)
  cv::Mat continuous = image.isContinuous() ? image : image.clone();
  sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
  msg->height = static_cast<std::uint32_t>(continuous.rows);
  msg->width = static_cast<std::uint32_t>(continuous.cols);
  msg->encoding = sensor_msgs::image_encodings::BGR8;
  msg->is_bigendian = 0;
  msg->step = static_cast<sensor_msgs::Image::_step_type>(continuous.step);
  msg->data.assign(continuous.datastart, continuous.dataend);
  msg->header.stamp = ros::Time(stamp);
  msg->header.frame_id = world_frame_id;
  publisher.publish(msg);
}

void FreedomViewer::publish_range_marker(
  double stamp,
  const FreedomBox& box,
  const std::string& ns,
  float r,
  float g,
  float b,
  ros::Publisher& publisher) {
  if (!box.valid) return;
  visualization_msgs::Marker msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = world_frame_id;
  msg.ns = ns;
  msg.id = 0;
  msg.type = visualization_msgs::Marker::CUBE;
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.position.x = (box.min.x() + box.max.x()) * 0.5;
  msg.pose.position.y = (box.min.y() + box.max.y()) * 0.5;
  msg.pose.position.z = (box.min.z() + box.max.z()) * 0.5;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = std::max(0.0, box.max.x() - box.min.x());
  msg.scale.y = std::max(0.0, box.max.y() - box.min.y());
  msg.scale.z = std::max(0.0, box.max.z() - box.min.z());
  msg.color.r = r;
  msg.color.g = g;
  msg.color.b = b;
  msg.color.a = 0.15f;
  publisher.publish(msg);
}

void FreedomViewer::process_scan_removal(const FreedomScanRemoval::ConstPtr& payload) {
  const double stamp = payload->stamp;
  if (payload->scan_blocks) publish_cloud(stamp, *payload->scan_blocks, scan_blocks_pub);
  if (payload->scan_voxels) publish_cloud(stamp, *payload->scan_voxels, scan_voxels_pub);
  if (payload->clusters) publish_cloud(stamp, *payload->clusters, clusters_pub);
  publish_range_marker(stamp, payload->scan_map_range, "scan_map_range", 0.0f, 1.0f, 0.0f, scan_map_range_pub);
}

void FreedomViewer::process_raycast_enhancement(const FreedomRaycastEnhancement::ConstPtr& payload) {
  const double stamp = payload->stamp;
  if (payload->depth_image && !payload->depth_image->empty())
    publish_image(stamp, *payload->depth_image, depth_image_pub);
  if (payload->enhanced_depth_image && !payload->enhanced_depth_image->empty()) {
    publish_image(stamp, *payload->enhanced_depth_image, enhanced_depth_image_pub);
  }
  if (payload->enhanced_pointcloud) {
    publish_cloud(stamp, *payload->enhanced_pointcloud, enhanced_pointcloud_pub);
  }
}

void FreedomViewer::process_map_removal(const FreedomMapRemoval::ConstPtr& payload) {
  const double stamp = payload->stamp;
  if (payload->raycasted_blocks) publish_cloud(stamp, *payload->raycasted_blocks, raycasted_blocks_pub);
  if (payload->raycasted_voxels) publish_cloud(stamp, *payload->raycasted_voxels, raycasted_voxels_pub);
  if (payload->free_blocks) publish_cloud(stamp, *payload->free_blocks, free_blocks_pub);
  if (payload->free_voxels) publish_cloud(stamp, *payload->free_voxels, free_voxels_pub);
  if (payload->static_blocks) publish_cloud(stamp, *payload->static_blocks, static_blocks_pub);
  if (payload->static_voxels) publish_cloud(stamp, *payload->static_voxels, static_voxels_pub);
  if (payload->static_subvoxels) publish_cloud(stamp, *payload->static_subvoxels, static_subvoxels_pub);
  if (payload->static_pointcloud) publish_cloud(stamp, *payload->static_pointcloud, static_pointcloud_pub);
  publish_range_marker(stamp, payload->raycast_range, "raycast_range", 0.0f, 0.5f, 1.0f, raycast_range_pub);
  publish_range_marker(stamp, payload->local_map_range, "local_map_range", 1.0f, 1.0f, 0.0f, local_map_range_pub);
}

}  // namespace asuka

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::FreedomViewer();
}

#include <asuka_ros/asuka_ros.hpp>

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <ros/package.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <asuka/utility/config.hpp>
#include <asuka/utility/config_impl.hpp>

namespace asuka {

namespace {
void warn_dropped_frame(const std::shared_ptr<spdlog::logger>& logger) {
  static std::atomic<int> dropped{0};
  const int count = ++dropped;
  if (count % 100 == 1) {
    logger->warn(
      "lidar frames dropped by the loaded odometry's preprocess ({} so far): "
      "check that config_odometry matches lidar_type",
      count);
  }
}
}  // namespace

AsukaROS::AsukaROS(ros::NodeHandle& nh) : nh(nh) {
  static std::atomic<int> live_instances{0};
  if (live_instances.fetch_add(1, std::memory_order_relaxed) != 0) {
    throw std::runtime_error(
      "second AsukaROS instance in this process: callback slots and global "
      "config are single-instance by design");
  }

  cv::setNumThreads(1);

  std::string config_dir;
  nh.getParam("config_path", config_dir);
  if (config_dir.empty()) {
    ros::NodeHandle("~").getParam("config_path", config_dir);
  }
  if (config_dir.empty()) {
    config_dir = ros::package::getPath("asuka") + "/config";
  }
  GlobalConfig::instance(config_dir);

  logger = create_module_logger("ros");

  std::shared_ptr<OdometryEstimation> odometry = OdometryEstimation::load_configured();
  cloud_preprocess = odometry->cloud_preprocess();
  async = std::make_unique<AsyncOdometryEstimation>(std::move(odometry));

  const Config ros_config = load_config("config_ros");
  const std::string lidar_type_str = ros_config.param<std::string>("ros", "lidar_type", "livox");
  lidar_type = parse_lidar_type(lidar_type_str);
  acc_scale = ros_config.param<double>("ros", "acc_scale", 1.0);
  acc_threshold = ros_config.param<double>("ros", "acc_threshold", 0.0);
  if (acc_threshold < 0) {
    throw std::runtime_error("config_ros: ros.acc_threshold must be non-negative (0 disables dropping)");
  }
  imu_time_offset = ros_config.param<double>("ros", "imu_time_offset", 0.0);
  time_offset_lidar_to_imu = ros_config.param<double>("ros", "time_offset_lidar_to_imu", 0.0);
  map_saving_path = ros_config.param<std::string>("ros", "map_saving_path", "");

  extension_modules = ExtensionModule::load_configured_modules();
}

AsukaROS::~AsukaROS() {
  if (async) async->stop();
  for (auto& ext : extension_modules) {
    ext->stop();
  }
  for (auto& ext : extension_modules) {
    try {
      ext->at_exit(map_saving_path);
    } catch (const std::exception& e) {
      logger->warn("extension at_exit failed: {}", e.what());
    } catch (...) {
      logger->warn("extension at_exit failed with an unknown exception");
    }
  }
}

void AsukaROS::insert_imu(const sensor_msgs::Imu::ConstPtr& msg) {
  ImuData::Ptr imu = std::make_shared<ImuData>();
  imu->stamp = msg->header.stamp.toSec();
  handle_loop_back(imu->stamp, FrameId::IMU);

  imu->linear_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu->angular_vel = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  imu->stamp += imu_time_offset;
  imu->linear_acc *= acc_scale;

  if (acc_threshold > 0 && imu->linear_acc.cwiseAbs().maxCoeff() > acc_threshold) {
    static std::atomic<int> dropped_imus{0};
    const int count = ++dropped_imus;
    if (count % 100 == 1) {
      logger->warn(
        "imu samples dropped by ros.acc_threshold ({} so far): check for sensor glitches "
        "or an over-tight threshold",
        count);
    }
    return;
  }
  async->insert_imu(imu);
}

void AsukaROS::insert_frame(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  const double stamp = cloud->header.stamp.toSec();
  switch (lidar_type) {
    case LidarType::LIVOX: {
      pcl::PointCloud<LivoxPoint>::Ptr raw(new pcl::PointCloud<LivoxPoint>());
      pcl::fromROSMsg(*cloud, *raw);
      insert_frame(stamp, raw);
      break;
    }
    case LidarType::ROBOSENSE: {
      pcl::PointCloud<RobosensePoint>::Ptr raw(new pcl::PointCloud<RobosensePoint>());
      pcl::fromROSMsg(*cloud, *raw);
      insert_frame(stamp, raw);
      break;
    }
    default:
      throw std::runtime_error(
        "lidar_type " + std::to_string(static_cast<int>(lidar_type)) + " does not consume sensor_msgs::PointCloud2");
  }
}

void AsukaROS::insert_frame(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
  const double stamp = msg->header.stamp.toSec();
  handle_loop_back(stamp, FrameId::LIDAR);
  const double adjusted_stamp = stamp + time_offset_lidar_to_imu;
  pcl::PointCloud<LivoxCustomPoint>::Ptr raw(new pcl::PointCloud<LivoxCustomPoint>());
  raw->resize(msg->points.size());
  for (std::size_t i = 0; i < msg->points.size(); ++i) {
    const auto& sp = msg->points[i];
    auto& p = raw->points[i];
    p.x = sp.x;
    p.y = sp.y;
    p.z = sp.z;
    p.intensity = sp.reflectivity;
    p.tag = sp.tag;
    p.line = sp.line;
    p.offset_time = sp.offset_time;
  }
  PointCloudT::Ptr processed = cloud_preprocess ? cloud_preprocess->preprocess(adjusted_stamp, raw) : nullptr;
  if (!processed) {
    warn_dropped_frame(logger);
    return;
  }
  async->insert_frame(adjusted_stamp, processed);
}

void AsukaROS::insert_frame(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr& raw) {
  handle_loop_back(stamp, FrameId::LIDAR);
  const double adjusted_stamp = stamp + time_offset_lidar_to_imu;
  PointCloudT::Ptr processed = cloud_preprocess ? cloud_preprocess->preprocess(adjusted_stamp, raw) : nullptr;
  if (!processed) {
    warn_dropped_frame(logger);
    return;
  }
  async->insert_frame(adjusted_stamp, processed);
}

void AsukaROS::insert_frame(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr& raw) {
  handle_loop_back(stamp, FrameId::LIDAR);
  const double adjusted_stamp = stamp + time_offset_lidar_to_imu;
  PointCloudT::Ptr processed = cloud_preprocess ? cloud_preprocess->preprocess(adjusted_stamp, raw) : nullptr;
  if (!processed) {
    warn_dropped_frame(logger);
    return;
  }
  async->insert_frame(adjusted_stamp, processed);
}

void AsukaROS::insert_frame(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr& raw) {
  handle_loop_back(stamp, FrameId::LIDAR);
  const double adjusted_stamp = stamp + time_offset_lidar_to_imu;
  PointCloudT::Ptr processed = cloud_preprocess ? cloud_preprocess->preprocess(adjusted_stamp, raw) : nullptr;
  if (!processed) {
    warn_dropped_frame(logger);
    return;
  }
  async->insert_frame(adjusted_stamp, processed);
}

void AsukaROS::insert_image(double stamp, const cv::Mat& image) {
  handle_loop_back(stamp, FrameId::CAMERA);
  auto data = std::make_shared<ImageData>();
  data->stamp = stamp;
  data->image = std::make_shared<const cv::Mat>(image);
  async->insert_image(data);
}

int AsukaROS::workload() const {
  return async ? async->workload() : 0;
}

bool AsukaROS::ok() const {
  if (!async || async->is_stopped()) return false;
  for (const auto& ext : extension_modules) {
    if (ext && !ext->ok()) return false;
  }
  return true;
}

void AsukaROS::wait(bool auto_quit) {
  if (!async) return;
  if (!auto_quit) {
    ros::WallRate rate(100);
    while (ros::ok()) {
      rate.sleep();
    }
    return;
  }
  while (ros::ok()) {
    if (async->wait_for(std::chrono::milliseconds(100))) break;
    if (!ok()) break;
  }
}

void AsukaROS::save(const std::string& path) {
  if (async) async->stop();
  auto cloud = async ? async->save_map() : nullptr;
  if (cloud && !cloud->empty() && !path.empty()) {
    std::filesystem::path fs_path(path);
    std::string file_path;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream stamp;
    stamp << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
    std::string filename = "mapping_" + stamp.str() + ".pcd";
    if (fs_path.has_extension()) {
      file_path = path;
    } else {
      std::filesystem::create_directories(path);
      file_path = path + "/" + filename;
    }
    try {
      using MapPoint = asuka::PointXYZIRGB;
      pcl::PointCloud<MapPoint>::Ptr cloud_map(new pcl::PointCloud<MapPoint>);
      cloud_map->resize(cloud->size());
      for (std::size_t i = 0; i < cloud->size(); ++i) {
        cloud_map->points[i].x = cloud->points[i].x;
        cloud_map->points[i].y = cloud->points[i].y;
        cloud_map->points[i].z = cloud->points[i].z;
        cloud_map->points[i].intensity = cloud->points[i].intensity;
        cloud_map->points[i].rgb = cloud->points[i].rgb;
      }
      pcl::io::savePCDFileBinary(file_path, *cloud_map);
      logger->info("saved map ({} points) to {}", cloud_map->size(), file_path);
    } catch (const pcl::IOException& e) {
      logger->warn("failed to save map: {}", e.what());
    }
  }
}

void AsukaROS::handle_loop_back(double stamp, const FrameId& frame_id) {
  if (frame_id == FrameId::LIDAR) {
    if (stamp < last_timestamp_lidar) {
      logger->warn("lidar loop back detected: {} -> {}", last_timestamp_lidar, stamp);
    }
    last_timestamp_lidar = stamp;
  } else if (frame_id == FrameId::IMU) {
    if (stamp < last_timestamp_imu) {
      logger->warn("imu loop back detected: {} -> {}", last_timestamp_imu, stamp);
    }
    last_timestamp_imu = stamp;
  } else if (frame_id == FrameId::CAMERA) {
    if (stamp < last_timestamp_camera) {
      logger->warn("camera loop back detected: {} -> {}", last_timestamp_camera, stamp);
    }
    last_timestamp_camera = stamp;
  }
}

}  // namespace asuka

#include <memory>
#include <string>

#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <livox_ros_driver/CustomMsg.h>

#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>

#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>
#include <asuka_ros/asuka_ros.hpp>

namespace asuka {

class AsukaNodelet final : public nodelet::Nodelet {
public:
  AsukaNodelet();
  ~AsukaNodelet() override;
  void onInit() override;

private:
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void custom_points_callback(const livox_ros_driver::CustomMsg::ConstPtr& msg);

private:
  std::string map_saving_path{};
  ros::Subscriber imu_sub{};
  ros::Subscriber points_sub{};
  ros::Subscriber image_sub{};
  ros::CallbackQueue callback_queue{};
  std::unique_ptr<ros::NodeHandle> node{nullptr};
  std::unique_ptr<ros::AsyncSpinner> spinner{nullptr};
  std::unique_ptr<AsukaROS> asuka_ros{nullptr};
  std::shared_ptr<spdlog::logger> logger{nullptr};
};

AsukaNodelet::AsukaNodelet() = default;

AsukaNodelet::~AsukaNodelet() {
  if (spinner) {
    spinner->stop();
  }
  if (asuka_ros) {
    asuka_ros->wait(true);
    asuka_ros->save(map_saving_path);
  }
}

void AsukaNodelet::onInit() {
  node = std::make_unique<ros::NodeHandle>(getPrivateNodeHandle());
  node->setCallbackQueue(&callback_queue);

  asuka_ros = std::make_unique<AsukaROS>(*node);
  logger = create_module_logger("ros");

  const Config config = load_config("config_ros");
  const std::string imu_topic = config.param<std::string>("ros", "imu_topic", "/livox/imu");
  const std::string lidar_topic = config.param<std::string>("ros", "lidar_topic", "/livox/lidar");
  map_saving_path = config.param<std::string>("ros", "map_saving_path", "");

  imu_sub = node->subscribe(imu_topic, 1000, &AsukaNodelet::imu_callback, this);
  const bool livox_custom =
    asuka::parse_lidar_type(config.param<std::string>("ros", "lidar_type", "livox")) == asuka::LidarType::LIVOX_CUSTOM;
  if (livox_custom) {
    points_sub = node->subscribe<livox_ros_driver::CustomMsg>(
      lidar_topic,
      50,
      boost::bind(&AsukaNodelet::custom_points_callback, this, _1));
  } else {
    points_sub = node->subscribe(lidar_topic, 50, &AsukaNodelet::points_callback, this);
  }

  const std::string image_topic = config.param<std::string>("ros", "image_topic", "");
  if (!image_topic.empty()) {
    image_sub = node->subscribe<sensor_msgs::Image>(image_topic, 10, [&](const sensor_msgs::ImageConstPtr& msg) {
      asuka_ros->insert_image(msg->header.stamp.toSec(), cv_bridge::toCvCopy(msg, "bgr8")->image);
    });
  }

  spinner = std::make_unique<ros::AsyncSpinner>(1, &callback_queue);
  spinner->start();

  logger->info("Asuka Nodelet initialized");
}

void AsukaNodelet::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  asuka_ros->insert_imu(msg);
}

void AsukaNodelet::points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  asuka_ros->insert_frame(msg);
}

void AsukaNodelet::custom_points_callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
  asuka_ros->insert_frame(msg);
}

}  // namespace asuka

PLUGINLIB_EXPORT_CLASS(asuka::AsukaNodelet, nodelet::Nodelet)

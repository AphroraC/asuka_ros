#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <livox_ros_driver/CustomMsg.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <asuka/utility/config.hpp>
#include <asuka_ros/asuka_ros.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "asuka_rosnode");
  ros::NodeHandle nh;

  asuka::AsukaROS asuka_ros(nh);

  const asuka::Config config = asuka::load_config("config_ros");
  const std::string imu_topic = config.param<std::string>("ros", "imu_topic", "/livox/imu");
  const std::string lidar_topic = config.param<std::string>("ros", "lidar_topic", "/livox/lidar");
  const std::string map_saving_path = config.param<std::string>("ros", "map_saving_path", "");
  const bool livox_custom =
    asuka::parse_lidar_type(config.param<std::string>("ros", "lidar_type", "livox")) == asuka::LidarType::LIVOX_CUSTOM;

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1000, [&](const sensor_msgs::Imu::ConstPtr& msg) {
    asuka_ros.insert_imu(msg);
  });

  ros::Subscriber points_sub;
  if (livox_custom) {
    points_sub =
      nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 50, [&](const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        asuka_ros.insert_frame(msg);
      });
  } else {
    points_sub =
      nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 50, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        asuka_ros.insert_frame(msg);
      });
  }

  const std::string image_topic = config.param<std::string>("ros", "image_topic", "");
  ros::Subscriber image_sub;
  if (!image_topic.empty()) {
    image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 10, [&](const sensor_msgs::ImageConstPtr& msg) {
      asuka_ros.insert_image(msg->header.stamp.toSec(), cv_bridge::toCvCopy(msg, "bgr8")->image);
    });
  }

  ros::spin();
  asuka_ros.wait(true);
  asuka_ros.save(map_saving_path);
  return 0;
}

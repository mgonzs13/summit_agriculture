#define BOOST_BIND_NO_PLACEHOLDERS

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>

#include "robotnik_filters/filter_node.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

FilterNode::FilterNode(const rclcpp::NodeOptions &options)
    : Node("FilterNode", options),
      publisher_{create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/output", rclcpp::QoS(1).reliable())},
      subscription_{create_subscription<sensor_msgs::msg::PointCloud2>(
          "~/input", rclcpp::QoS(1).reliable(),
          std::bind(&FilterNode::topic_callback, this,
                    std::placeholders::_1))} {}

void FilterNode::topic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // ROS2 message to PCL message
  pcl::PCLPointCloud2::Ptr cloud_filtered{new pcl::PCLPointCloud2{}};
  pcl_conversions::toPCL(*msg, *cloud_filtered);

  Eigen::Vector4f const minPoint{-0.5, -0.46, -1.0, 1.0};
  Eigen::Vector4f const maxPoint{-0.05, 0.46, 1.0, 1.0};

  // Remove points inside the box
  pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
  boxFilter.setInputCloud(cloud_filtered);
  boxFilter.setMin(minPoint);
  boxFilter.setMax(maxPoint);
  boxFilter.setNegative(true);
  boxFilter.filter(*cloud_filtered);

  // Outlier removal filter
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(3);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  // PCL message to ROS2 message
  sensor_msgs::msg::PointCloud2 cloud_out;
  pcl_conversions::fromPCL(*cloud_filtered, cloud_out);

  cloud_out.header.frame_id = msg->header.frame_id;
  cloud_out.header.stamp = msg->header.stamp;
  publisher_->publish(cloud_out);
}

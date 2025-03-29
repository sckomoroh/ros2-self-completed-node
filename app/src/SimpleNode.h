/**
 * @file SimpleNode.h
 * @brief Entry point for the ROS2 Simple Node application.
 * 
 * @author Yehor Zvihunov
 * @date 30.03.2025
 */

 #pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <rclcpp/executors/single_threaded_executor.hpp>

namespace ros2_simple::node {

class SimpleNode {
private:
    rclcpp::Node::SharedPtr _node;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> _pointCloudSubscriber;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> _imageSubscriber;

    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage>
        _syncPolicy;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                                                  sensor_msgs::msg::CompressedImage>>
        _synchronizer;

    rclcpp::executors::SingleThreadedExecutor _nodeExecutor;

    uint8_t _messageCounts = 0; // Just for stop demonstarion
    
public:
    SimpleNode();

public:
    void spinNode();

private:
    void synchronizerCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloud,
                              sensor_msgs::msg::CompressedImage::ConstSharedPtr image);
};

}  // namespace ros2_simple::node
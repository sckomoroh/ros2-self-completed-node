/**
 * @file SimpleNode.cpp
 * @brief Entry point for the ROS2 Simple Node application.
 * 
 * @author Yehor Zvihunov
 * @date 30.03.2025
 */

 #include "SimpleNode.h"

#include <rmw/qos_profiles.h>

namespace ros2_simple::node {

namespace {

constexpr const char* kPointCloudTopic = "";
constexpr const char* kImageTopic = "";

}  // namespace

SimpleNode::SimpleNode()
    : _syncPolicy{10}
    , _synchronizer{_syncPolicy} {
    _node = rclcpp::Node::make_shared("simple_node");

    _nodeExecutor.add_node(_node);

    _pointCloudSubscriber.subscribe(_node, kPointCloudTopic, rmw_qos_profile_default);
    _imageSubscriber.subscribe(_node, kImageTopic, rmw_qos_profile_default);

    _syncPolicy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.5));

    _synchronizer.connectInput(_pointCloudSubscriber, _imageSubscriber);

    _synchronizer.registerCallback(
        std::bind(&SimpleNode::synchronizerCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(_node->get_logger(), "Initialization completed");
}

void SimpleNode::spinNode() { _nodeExecutor.spin(); }

void SimpleNode::synchronizerCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloud,
                                      sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {
    _messageCounts++;

    double pointCloudTimestamp = pointCloud->header.stamp.sec + pointCloud->header.stamp.nanosec / 1e9;
    double imageTimestamp = image->header.stamp.sec + image->header.stamp.nanosec / 1e9;

    RCLCPP_INFO(_node->get_logger(),
                "Received synchronized messages. Count = %u, PointCloud timestamp: %.9f, Image timestamp: %.9f",
                _messageCounts,
                pointCloudTimestamp,
                imageTimestamp);

    if (_messageCounts == 100) {
        RCLCPP_INFO(_node->get_logger(), "Target messages receiver. Stop node.");
        _nodeExecutor.cancel();
    }
}

}  // namespace ros2_simple::node

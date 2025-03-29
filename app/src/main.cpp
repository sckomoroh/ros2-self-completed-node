#include "SimpleNode.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = ros2_simple::node::SimpleNode();
    node.spinNode();

    return 0;
}
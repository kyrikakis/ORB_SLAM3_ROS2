#ifndef __IMAGE_STREAM_NODE_HPP__
#define __IMAGE_STREAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "utility.hpp"

class ImageStreamNode : public rclcpp::Node
{
public:
    ImageStreamNode();

    ~ImageStreamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void StreamImage();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
    //rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;
};

#endif

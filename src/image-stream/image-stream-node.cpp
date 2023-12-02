#include "image-stream-node.hpp"

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>

using std::placeholders::_1;

ImageStreamNode::ImageStreamNode()
:   Node("camera"), cim(this)
{
    size_t depth = 5;
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
    qos_profile.depth = depth;

    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos_profile.reliability = reliability_policy;

    // The history policy determines how messages are saved until the message is taken by the reader.
    // KEEP_ALL saves all messages until they are taken.
    // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
    // parameter.
    qos_profile.history = history_policy;

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile));
    
    pub_image = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);

    if (!cim.setCameraName("pi_module3_wide"))
        throw std::runtime_error("camera name must only contain alphanumeric characters");
    ImageStreamNode::StreamImage();
}

void ImageStreamNode::StreamImage() 
{
    std::string serverAddress = "192.168.1.17";
    unsigned short serverPort = 8888;

    // Setup boost::asio
    boost::asio::io_service ioService;
    boost::asio::ip::tcp::socket socket(ioService);
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(serverAddress), serverPort);

    try {
        socket.connect(endpoint);
        while (true) {
            // // send image data
            std_msgs::msg::Header header;
            header.frame_id = "camera";
            header.stamp = this->get_clock()->now();

            // Read the length of the next frame (assuming it's a 4-byte integer)
            uint32_t lenFrame;
            boost::asio::read(socket, boost::asio::buffer(&lenFrame, sizeof(lenFrame)));
            // Read the frame data
            std::vector<char> frameData(lenFrame);
            boost::asio::read(socket, boost::asio::buffer(frameData.data(), lenFrame));

            // Decode the frame using OpenCV
            cv::Mat frame = cv::imdecode(cv::Mat(frameData), cv::IMREAD_COLOR);

            if (frame.empty()) {
                std::cerr << "Error decoding frame." << std::endl;
                continue;
            }

            cv_bridge::CvImage img_bridge;

            img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);
            ImageMsg::SharedPtr msg_img = img_bridge.toImageMsg();
            pub_image->publish(*msg_img); 

            sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
            ci.header = header;
            pub_ci->publish(ci);
        }
        // Clean up
        cv::destroyAllWindows();
    } catch (cv::Exception& e) {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return;
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

ImageStreamNode::~ImageStreamNode()
{

}
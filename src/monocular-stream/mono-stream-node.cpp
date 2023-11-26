#include "mono-stream-node.hpp"

#include<opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using std::placeholders::_1;

MonoStreamNode::MonoStreamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    size_t depth = 10;
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

    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    // m_image_subscriber = this->create_subscription<ImageMsg>(
    //     "image",
    //     qos,
    //     std::bind(&MonoStreamNode::GrabImage, this, std::placeholders::_1));
    // subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonoStreamNode::GrabImu, this, _1));
    // syncThread_ = new std::thread(&MonoStreamNode::SyncWithImu, this);
    std::cout << "slam changed" << std::endl;
    MonoStreamNode::startStream();
}

MonoStreamNode::~MonoStreamNode()
{
    MonoStreamNode::stream = false;
    // Stop all threads
    // m_SLAM->Shutdown();

    // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoStreamNode::startStream() 
{
    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cap.open("tcp://192.168.1.210:8888");

    std::cout << "buffer size: " << cap.get(cv::CAP_PROP_BUFFERSIZE) << std::endl;

    // Check if the video capture object is successfully opened
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream from TCP source" << std::endl;
        return;
    } else {
        std::cout << "video capture suceeded!" << std::endl;
    }

    cv::Mat frame;
    cap >> frame;
    std::cout << "frame received size: " << frame.size() << std::endl;

    //cv::namedWindow("Video Stream", cv::WINDOW_NORMAL);

    try {
        while (MonoStreamNode::stream) {
            // Read a frame from the video source
            cap >> frame;

            // Check if the frame is empty (end of video stream)
            if (frame.empty()) {
                std::cout << "End of video stream" << std::endl;
                break;
            } else {
                std::cout << "frame rate: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
            }

            // Display the frame (you can replace this with your processing logic)
            //cv::imshow("Video Stream", frame);
            // Check for errors
            m_SLAM->TrackMonocular(frame, Utility::StampToSec(this->get_clock()->now())); 
        }
        // Clean up
        cv::destroyAllWindows();
    } catch (cv::Exception& e) {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return;
    }
}

void MonoStreamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoStreamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msg);

    bufMutexLeft_.unlock();

}

cv::Mat MonoStreamNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoStreamNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft;
        double tImLeft = 0;
        if (!imgLeftBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);

            bufMutexLeft_.lock();
            while (imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            std::cout<<"one frame has been sent"<<std::endl;
            m_SLAM->TrackMonocular(imLeft, tImLeft, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

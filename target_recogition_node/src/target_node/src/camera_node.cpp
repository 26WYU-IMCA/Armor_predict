#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // 初始化节点
        RCLCPP_INFO(this->get_logger(), "Camera node starting...");

        // 从参数中获取视频路径
        std::string video_path = this->declare_parameter<std::string>("video_path", "/home/x/visionlib/vision.mp4"); 

        // 打开视频文件或摄像头
        cap_ = cv::VideoCapture(video_path);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source: %s", video_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // 获取视频帧率
        double fps = cap_.get(cv::CAP_PROP_FPS);
        if (fps <= 0)
        {
            fps = 30.0; // 默认帧率为30fps
        }
        int timer_period_ms = static_cast<int>(1000.0 / fps);

        // 创建发布者
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

        // 创建定时器
        timer_ = this->create_wall_timer(timer_period_ms * 1ms, std::bind(&CameraNode::capture_frame, this));
    }

private:
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void capture_frame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
            return;
        }
        if (cv::waitKey(1) == 27)
        {
            rclcpp::shutdown();
        }
        // 将cv::Mat转换为sensor_msgs::msg::Image
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera";

        try
        {
            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub_->publish(*msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
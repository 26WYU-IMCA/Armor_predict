#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <algorithm>
#include "Target.h"
#include "Kalman.h"
#include "node_parameter/srv/myparameter.hpp"

using namespace std::chrono_literals;

bool compare(armorDetector::Light left_light, armorDetector::Light right_light)
{
    return right_light.light.center.x > left_light.light.center.x;
}

class ImageReceiverNode : public rclcpp::Node
{
public:
    ImageReceiverNode() : Node("image_receiver"), find(15, 10, 2.5)
    {
        std::cout << "Initializing ImageReceiverNode..." << std::endl;                                                          
        std::string topic_name = this->declare_parameter<std::string>("topic_name", "camera/image"); // 从参数中获取订阅的主题名称
        std::string srv_name = this->declare_parameter<std::string>("srv_name", "predict");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic_name, 10, std::bind(&ImageReceiverNode::image, this, std::placeholders::_1)); // 创建订阅者
        armor_send = this->create_client<node_parameter::srv::Myparameter>(srv_name);
        cv::namedWindow("video", cv::WINDOW_AUTOSIZE); // 创建OpenCV窗口
        RCLCPP_INFO(this->get_logger(), "wait for srv");
    }

private:
    void predcit_x(rclcpp::Client<node_parameter::srv::Myparameter>::SharedFuture x_);
    void armor_track(const sensor_msgs::msg::Image::SharedPtr msg);
    void image(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Client<node_parameter::srv::Myparameter>::SharedPtr armor_send;
    armorDetector::Camera_cailbration camera;
    armorDetector find;
    float x_pre = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageReceiverNode>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

void ImageReceiverNode::armor_track(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat src, edges, dst;
    try
    {
        // 使用cv_bridge将ROS图像消息转换为OpenCV格式
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
        src = cv_image->image;
        if (src.empty())
        {
            std::cout << "NO FIND SRC" << std::endl;
            return;
        }
        cv::undistort(src, dst, camera.camera_matrix, camera.distortion_coefficients); // 图像校正

        find.ProcessImage(dst, edges, 145); // 图像处理  输入图像  输出图像   二值化的阈值
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<armorDetector::Light> light_Rects;
        for (int i = 0; i < contours.size(); ++i)
        {
            cv::RotatedRect minRects = cv::minAreaRect(cv::Mat(contours[i]));
            if (find.IsLight(minRects)) // 判断是否是符合条件的灯条
            {
                armorDetector::Light minRects_ = {minRects, minRects.size.width * minRects.size.height, minRects.angle};
                light_Rects.push_back(minRects_);
            }
        }

        std::sort(light_Rects.begin(), light_Rects.end(), compare); // 灯条按x的大小降序排序
        // cv::Mat measurement;
        if (light_Rects.size() >= 2)
        {
            for (int i = 0; i + 1 < light_Rects.size(); i++)
            {
                armorDetector::Light left_light = light_Rects[i];
                armorDetector::Light right_light = light_Rects[i + 1];
                if (find.MatchArmor(left_light, right_light)) // 如果是装甲板就标记
                {
                    armorDetector::Armor target_armor = {left_light, right_light};
                    find.x_y_(target_armor);
                    // if (find.update_armor == 0)
                    // {
                    //     find.new_armor = target_armor;
                    //     find.last_armor = target_armor;
                    //     find.update_armor = 1;
                    //     //kf.kalman.statePost = (cv::Mat_<float>(3, 1) << target_armor.x, 0, 0);
                    // }
                    // else
                    // {
                    //     //find.new_armor.armor_speed = kf.kalman.statePost.at<float>(1);
                    //     find.last_armor = find.new_armor;
                    //     find.new_armor = target_armor;
                    //     if (!find.same_armor())
                    //     {
                    //         find.last_armor = target_armor;
                    //         //kf.kalman.statePost.at<float>(0) = target_armor.x; // 更新位置    singer
                    //         //kf.kalman.statePost.at<float>(1) = 0;              // 重置速度    singer
                    //     }
                    // }
                    // measurement = (cv::Mat_<float>(1, 1) << target_armor.x);
                    find.DrawArmor(dst, target_armor);
                    // kf.kalman_predict();
                    // kf.kalman_updata(measurement);
                    // float x_ = kf.kalman.statePost.at<float>(0);
                    //  std::cout<<"statePost: "<<kf.kalman.statePost<<std::endl;
                    //  std::cout<<"measurement: "<< target_armor.x <<std::endl;
                    // cv::circle(dst, cv::Point(x_, target_armor.y), 10, cv::Scalar(0, 0, 255), 5);
                    i++;
                }
            }
        }
        if (cv::waitKey(1) == 27)
        {
            rclcpp::shutdown();
        }
        cv::imshow("video", dst);

        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void ImageReceiverNode::image(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat dst, src, edges;
    try
    {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
        src = cv_image->image;
        cv::undistort(src, dst, camera.camera_matrix, camera.distortion_coefficients);
        find.ProcessImage(dst, edges, 145);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<armorDetector::Light> light_Rects;
        for (int i = 0; i < contours.size(); ++i)
        {
            cv::RotatedRect minRects = cv::minAreaRect(cv::Mat(contours[i]));
            if (find.IsLight(minRects)) // 判断是否是符合条件的灯条
            {
                armorDetector::Light minRects_ = {minRects, minRects.size.width * minRects.size.height, minRects.angle};
                light_Rects.push_back(minRects_);
            }
        }
        std::sort(light_Rects.begin(), light_Rects.end(), compare);
        cv::Mat measurement;
        if (light_Rects.size() >= 2)
        {
            for (int i = 0; i + 1 < light_Rects.size(); i++)
            {
                armorDetector::Light left_light = light_Rects[i];
                armorDetector::Light right_light = light_Rects[i + 1];
                if (find.MatchArmor(left_light, right_light)) // 如果是装甲板就标记
                {
                    armorDetector::Armor target_armor = {left_light, right_light};
                    find.x_y_(target_armor);
                    auto x_y_z = std::make_shared<node_parameter::srv::Myparameter_Request>();
                    x_y_z->x = target_armor.x;
                    x_y_z->y = target_armor.y;
                    x_y_z->z = target_armor.z;
                    armor_send->async_send_request(x_y_z, std::bind(&ImageReceiverNode::predcit_x, this, std::placeholders::_1));
                    //find.DrawArmor(dst, target_armor);
                    cv::RotatedRect left = target_armor.left_light.light;
                    cv::RotatedRect right = target_armor.right_light.light;
                    std::vector<cv::Point2f> leftxy(4), rightxy(4);
                    right.points(rightxy.data());
                    left.points(leftxy.data());
                    cv::Point2f a1 = (leftxy[1] + leftxy[2]) / 2;
                    cv::Point2f a2 = (leftxy[0] + leftxy[3]) / 2;
                    cv::Point2f b1 = (rightxy[1] + rightxy[2]) / 2;
                    cv::Point2f b2 = (rightxy[0] + rightxy[3]) / 2;
                    if (a1.y < a2.y)
                    {
                        auto leftRect = a1;
                        a1 = a2;
                        a2 = leftRect;
                    }
                    if (b1.y < b2.y)
                    {
                        auto rightRect = b1;
                        b1 = b2;
                        b2 = rightRect;
                    }
                    cv::line(dst, a1, b2, cv::Scalar(0, 255, 0), 3);
                    cv::line(dst, a2, b1, cv::Scalar(0, 255, 0), 3);
                    //RCLCPP_INFO(this->get_logger(), "x value: %f", x_pre);
                    if (x_pre!= 0)
                    {
                        cv::circle(dst, cv::Point(x_pre, target_armor.y), 10, cv::Scalar(0, 0, 255), 5);
                    }
                    i++;
                }
            }
        }
        cv::imshow("video", dst);
        cv::waitKey(1);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}

void ImageReceiverNode::predcit_x(rclcpp::Client<node_parameter::srv::Myparameter>::SharedFuture x_) // 回应函数
{
    auto x = x_.get();
    x_pre = x->x_a;
    //RCLCPP_INFO(this->get_logger(), "x value: %f", x_pre);
}

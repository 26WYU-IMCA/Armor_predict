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
class kalman_predict : public rclcpp::Node
{
public:
    // 声明服务
    kalman_predict() : Node("kalman")
    {
        RCLCPP_INFO(this->get_logger(), "predict strating");
        std::string srv_name = this->declare_parameter<std::string>("srv_name", "predict");
        kf.init(3, 1, 0);
        kf.kfinit_uniform();
        predict_update = this->create_service<node_parameter::srv::Myparameter>(srv_name, std::bind(&kalman_predict::predict, this, std::placeholders::_1, std::placeholders::_2)); // 创建服务端并处理请求
    }

private:
    rclcpp::Service<node_parameter::srv::Myparameter>::SharedPtr predict_update;
    void predict(const node_parameter::srv::Myparameter::Request::SharedPtr request, const node_parameter::srv::Myparameter::Response::SharedPtr response); // 处理请求
    kalman_armor kf;
};

void kalman_predict::predict(const node_parameter::srv::Myparameter::Request::SharedPtr request, const node_parameter::srv::Myparameter::Response::SharedPtr response)
{
    cv::Mat measurement;
    kf.target_armor.x = request->x;
    kf.target_armor.y = request->y;
    kf.target_armor.z = request->z;
    //RCLCPP_INFO(this->get_logger(), "x value: %f", request->x);
    if (kf.update_armor == 0)
    {
        kf.new_armor = kf.target_armor;
        kf.last_armor = kf.target_armor;
        kf.update_armor = 1;
        kf.kalman.statePost = (cv::Mat_<float>(3, 1) << kf.target_armor.x, 0, 0);
    }
    else
    {
        kf.new_armor.armor_speed = kf.kalman.statePost.at<float>(1);
        kf.last_armor = kf.new_armor;
        kf.new_armor = kf.target_armor;
        if (!kf.same_armor())
        {
            kf.last_armor = kf.target_armor;
            kf.kalman.statePost.at<float>(0) = kf.target_armor.x; // 更新位置    singer
            kf.kalman.statePost.at<float>(1) = 0;                 // 重置速度    singer
        }
    }
    measurement = (cv::Mat_<float>(1, 1) << kf.target_armor.x);
    kf.kalman_predict();
    kf.kalman_updata(measurement);
    float x_ = kf.kalman.statePost.at<float>(0);
    //RCLCPP_INFO(this->get_logger(), "x value: %f", x_);
    float y_ = kf.target_armor.y;
    float z_ = kf.target_armor.z;
    response->x_a = x_;
    response->y_b = y_;
    response->z_b = z_;
}

int main(int agrc, char **agrv) // 初始化节点并运行
{
    rclcpp::init(agrc, agrv);
    auto node = std::make_shared<kalman_predict>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

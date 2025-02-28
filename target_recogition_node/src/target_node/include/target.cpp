// Author:xpoying
// 函数定义   预处理图像，转为灰度图后二值化，开运算，筛选出灯条，根据其匹配程度（角度差，面积差，水平程度）找对应的装甲板

#include "Target.h"
#include <opencv2/video/tracking.hpp>
armorDetector::armorDetector(double max_height_difference, double max_length_width_ratio, double min_length_width_ratio) : max_height_difference_(max_height_difference),
                                                                                                                           max_length_width_ratio_(max_length_width_ratio),
                                                                                                                           min_length_width_ratio_(min_length_width_ratio) {};
// 图像处理
void armorDetector::ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold)
{
    cv::Mat thr, gray, dil;
    //cv::Size ken_size(3, 3); 
    //cv::Mat ken = cv::getStructuringElement(cv::MORPH_RECT, ken_size);
    double threshold1 = 3;
    double threshold2 = 9;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thr, threshold, 255, cv::THRESH_BINARY);
    //cv::morphologyEx(thr, dil, cv::MORPH_OPEN, ken, cv::Point(-1, -1));
    cv::Canny(thr, edges, threshold1, threshold2);
}

// 灯条判断
bool armorDetector::IsLight(cv::RotatedRect &minRects)
{
    if (minRects.size.width > minRects.size.height)
    {
        minRects.angle += 90;
        float t = minRects.size.width;
        minRects.size.width = minRects.size.height;
        minRects.size.height = t;
    }
    if ((minRects.size.height / minRects.size.width < max_length_width_ratio_) && (min_length_width_ratio_ < minRects.size.height / minRects.size.width))
    {
        return true;
    }
    else
        return false;
}

// 装甲板标记
void armorDetector::DrawArmor(const cv::Mat &image, Armor &target_Armor)
{
    cv::RotatedRect left = target_Armor.left_light.light;
    cv::RotatedRect right = target_Armor.right_light.light;
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

    cv::line(image, a1, b2, cv::Scalar(0, 255, 0), 3);
    cv::line(image, a2, b1, cv::Scalar(0, 255, 0), 3);
}

// 装甲板匹配
bool armorDetector::MatchArmor(const armorDetector::Light &right_light, const armorDetector::Light &left_light)
{
    double half_height = (right_light.light.size.height + left_light.light.size.height) / 4;
    // if (abs(right_light.angle - left_light.angle) > 90)    //负优化
    // return false;
    if (std::min(right_light.area, left_light.area) * 4 < std::max(right_light.area, left_light.area)) // 面积差
        return false;
    else if (abs(left_light.light.center.y - right_light.light.center.y) > 0.8 * half_height) // 水平差
        return false;
    else if (abs(left_light.light.center.y - right_light.light.center.y) > max_height_difference_) // y轴差
        return false;
    else
        return true;
}

// 三维坐标获取
void armorDetector::x_y_(armorDetector::Armor &armor_)
{
    double armor_pixel_coordinate_X = (armor_.left_light.light.center.x + armor_.right_light.light.center.x) / 2;
    double armor_pixel_coordinate_Y = (armor_.left_light.light.center.y + armor_.right_light.light.center.y) / 2;
    armorDetector::Camera_cailbration cam_;
    double f_x = cam_.camera_matrix.at<double>(0, 0);
    double c_x = cam_.camera_matrix.at<double>(0, 2);
    double f_y = cam_.camera_matrix.at<double>(1, 1);
    double c_y = cam_.camera_matrix.at<double>(1, 2);
    double armor_z = f_x * armor_.armor_width / abs(armor_.left_light.light.center.x - armor_.right_light.light.center.x);
    armor_.z = armor_z;
    armor_.x = armor_pixel_coordinate_X;
    armor_.y = armor_pixel_coordinate_Y;
    // armor_.armor_angle = atan((armor_.y - last_armor.y / (armor_.x - last_armor.x)) * 180 / 3.1416);
}

// void armorDetector::tack_armor(const armorDetector::Light &right_light, const armorDetector::Light &left_light)
// {
//     switch (state)
//     {
//     case State::SEARCHING_STATE:
//         if (MatchArmor(right_light,left_light))
//         {
//             state = State::TRACKING_STATE;
//             tacking_cnt = 0;
//         }
//         break;
//     case State::TRACKING_STATE:

//     break;
//     }
// }

bool armorDetector::same_armor()
{
    // 判断两个装甲板是否为同一个目标
    // 如果 x 坐标之差小于 10，则认为是同一个目标
    return abs(last_armor.x - new_armor.x) < 50;
}
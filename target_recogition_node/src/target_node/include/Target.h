// Author:xpoying
// 头文件 装甲板结构体，装甲板识别类，标记装甲板的函数声明
#ifndef TARGET_H
#define TARGET_H
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <opencv2/video/tracking.hpp>
class armorDetector
{
public:
    struct Light
    {
        cv::RotatedRect light;
        double area;
        double angle;
    };

    struct Armor
    {
        armorDetector::Light left_light;
        armorDetector::Light right_light;
        float armor_angle;
        float armor_speed;
        int armor_height = 127; // cm
        int armor_width = 230;  // cm
        float x;
        float y;
        float z;
    };
    struct Camera_cailbration
    {
        // 相机内参
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1034.174789, 0.000000, 638.103833,
                                 0.000000, 1034.712789, 494.576371,
                                 0.000000, 0.000000, 1.000000);
        // 畸变矩阵
        cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << -0.100831, 0.094629, -0.000572, 0.000629, 0.000000);

        // 校正矩阵
        cv::Mat rectification_matrix = (cv::Mat_<double>(3, 3) << 1.000000, 0.000000, 0.000000,
                                        0.000000, 1.000000, 0.000000,
                                        0.000000, 0.000000, 1.000000);
        // 投影矩阵
        cv::Mat projection_matrix = (cv::Mat_<double>(3, 4) << 1008.22804, 0.000000, 638.99992,
                                     0.000000, 0.000000, 1014.43577,
                                     493.63869, 0.000000, 0.000000,
                                     0.000000, 1.000000, 0.000000);
    };
    explicit armorDetector(double max_height_difference, double max_length_width_ratio, double min_length_width_ratio);
    void ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold);
    void DrawArmor(const cv::Mat &image, armorDetector::Armor &target_Armor);
    bool IsLight(cv::RotatedRect &minRects);
    bool MatchArmor(const armorDetector::Light &right_light, const armorDetector::Light &left_light);
    void x_y_(armorDetector::Armor &armor_);
    void tack_armor(const armorDetector::Light &right_light, const armorDetector::Light &left_light);
    bool same_armor();
    int tacking_cnt; // 记录追踪帧数，用于定时退出追踪
    Armor last_armor,new_armor;
    bool update_armor = 0;
private:
    double max_height_difference_;
    double max_length_width_ratio_;
    double min_length_width_ratio_;

    enum class State     // 自瞄状态枚举定义
    {
        SEARCHING_STATE, // 搜索状态
        TRACKING_STATE,  // 跟踪状态
        STANDBY_STATE,   // 待机状态
    };
    State state;         // 自瞄状态对象实例
    bool find_armor = 0;
};

#endif

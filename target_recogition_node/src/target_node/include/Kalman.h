#ifndef KALMAN_H
#define KALMAN_H
#include <opencv2/opencv.hpp>
#include"Target.h"



class kalman_armor
{
public:
    armorDetector::Armor last_armor,new_armor,target_armor;
    bool update_armor = 0;
    bool same_armor();
    void init(int DP_, int MP_, int CP_);                                 //卡尔曼滤波初始化     
    void singer_init(float alpha, float dt, float p, float k, float r);   //机动追踪模型初始化
    void kalman_predict();                                                //卡尔曼滤波预测     
    void kalman_updata(cv::Mat measureMat_last);                          //卡尔曼滤波更新预测值
    float singer[5];                                                      //机动追踪模型参数
    int DP;
    int MP;
    int CP;
    cv::Mat statePre;         // 预测状态向量 x'(k)
    cv::Mat stateOpt;         // 修正状态向量x(k)
    cv::Mat transMat;         // 状态转移矩阵A
    cv::Mat measureMat;       // 测量矩阵H
    cv::Mat processNoiseCov;  // 过程噪声协方差Q
    cv::Mat measureNosiseCov; // 测量噪声协方差R
    cv::Mat errorCovpre;      // 先验误差协方差P'(k)
    cv::Mat errorCovOpt;      // 后验误差协方差P(k)
    cv::Mat kgain;            // 卡尔曼增益
    cv::Mat contrMat;         // 控制矩阵B
    cv::KalmanFilter kalman;  //卡尔曼滤波
    
    void kfinit_uniform();    //匀加速模型
    void kfinit_singer();     //机动追踪模型
};

#endif
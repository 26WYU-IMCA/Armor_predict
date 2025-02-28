#include"Kalman.h"

void kalman_armor::init(int DP, int MP, int CP)
{
    // assert(DP > 0 && MP > 0);
    // CP = std::max(CP, 0);
    statePre = cv::Mat::zeros(DP, 1, CV_32F); // 预测状态向量 x'(k)
    stateOpt = cv::Mat::zeros(DP, 1, CV_32F); // 修正状态向量x(k)
    // transMat = cv::Mat::eye(DP, DP, CV_32F);         // 状态转移矩阵A
    // measureMat = cv::Mat::zeros(MP, DP, CV_32F); // 测量矩阵H
    // processNoiseCov = cv::Mat::eye(DP, DP, CV_32F);  // 过程噪声协方差Q
    // measureNosiseCov = cv::Mat::eye(MP, MP, CV_32F); // 测量噪声协方差R
    // errorCovpre = cv::Mat::eye(DP, DP, CV_32F);      // 先验误差协方差P'(k)
    // errorCovOpt = cv::Mat::eye(DP, DP, CV_32F);      // 后验误差协方差P(k)
    // kgain = cv::Mat::zeros(DP, MP, CV_32F);          // 卡尔曼增益

    // if (CP > 0)
    // {
    //     contrMat = cv::Mat::zeros(DP, CP, CV_32F); // 控制矩阵
    // }
    kalman.init(DP, MP, CP, CV_32F);
}
void kalman_armor::singer_init(float alpha, float dt, float p, float k, float r)
{
    singer[0] = alpha;
    singer[1] = dt;
    singer[2] = p;
    singer[3] = k;
    singer[4] = r;
}
void kalman_armor::kfinit_singer()
{
    // 初始化卡尔曼滤波器，状态维度为3，测量维度为3，控制维度为1
    float alpha = singer[0]; // 参数 alpha
    float dt = singer[1];    // 时间步长 dt

    // 初始化状态转移矩阵 A
    kalman.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
                                  0, 1, (1 - exp(-alpha * dt)) / alpha,
                                  0, 0, exp(-alpha * dt));

    // 初始化测量矩阵 H
    kalman.measurementMatrix = (cv::Mat_<float>(1, 3) << 1, 0, 0);
    // 初始化控制矩阵 B
    kalman.controlMatrix = (cv::Mat_<float>(3, 1) << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                               dt - (1 - exp(-alpha * dt) / alpha),
                               1 - exp(-alpha * dt));

    // 初始化后验误差协方差矩阵 P(k)
    float p = singer[2];
    kalman.errorCovPost = (cv::Mat_<float>(3, 3) << p, 0, 0,
                              0, p, 0,
                              0, 0, p);

    // 计算过程噪声协方差矩阵 Q
    float q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
    float q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
    float q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
    float q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
    float q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
    float q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));

    float k = singer[3];
    kalman.processNoiseCov = (cv::Mat_<float>(3, 3) << k * alpha * q11, k * alpha * q12, k * alpha * q13,
                                 k * alpha * q12, k * alpha * q22, k * alpha * q23,
                                 k * alpha * q13, k * alpha * q23, k * alpha * q33);

    // 初始化测量噪声协方差矩阵 R
    float r = singer[4];
    kalman.measurementNoiseCov = r * cv::Mat::eye(1, 1, CV_32F);
}

void kalman_armor::kalman_predict()
{
    statePre = kalman.predict();
}

void kalman_armor::kalman_updata(cv::Mat measureMat_last)
{
    stateOpt = kalman.correct(measureMat_last);
}

//----------------------------------------------------------------------------------------------------------
void kalman_armor::kfinit_uniform()
{
    float dt = 1.0f;                 // 时间步长 dt
    float process_noise = 1e-6f;     // 过程噪声
    float measurement_noise = 1e-2f; // 测量噪声
    float error_covariance = 1e-2f;  // 初始误差协方差
    // 使用匀加速模型
    kalman.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, dt, 0.5f * dt * dt,
                                                         0, 1, dt,
                                                          0, 0, 1);

    kalman.measurementMatrix = (cv::Mat_<float>(1, 3) << 1, 0, 0);

    // 初始化后验误差协方差矩阵 P(k)
    kalman.errorCovPost = cv::Mat::eye(3, 3, CV_32F) * error_covariance;

    // 初始化过程噪声协方差矩阵 Q
    kalman.processNoiseCov = cv::Mat::eye(3, 3, CV_32F) * process_noise;

    // 初始化测量噪声协方差矩阵 R
    kalman.measurementNoiseCov = measurement_noise * cv::Mat::eye(1, 1, CV_32F);
}


bool kalman_armor::same_armor()
{
    // 判断两个装甲板是否为同一个目标
    // 如果 x 坐标之差小于 10，则认为是同一个目标
    return abs(last_armor.x - new_armor.x) < 50;
}
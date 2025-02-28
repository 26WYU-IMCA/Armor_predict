# Armor_Predict
本项目为装甲板预测部分。项目基于ubuntu平台使用VScode开发，主要用于Robomaster比赛中测试各机动目标模型对装甲板预测的效果以及相关参数调试。
### 功能实现
* 基于特征匹配的装甲板识别功能
* 基于Singer模型和匀加速模型的装甲板X轴像素坐标卡尔曼预测
* 实时显示预测坐标
* 保存预测参数并图线绘制<br/>
### 依赖环境
#### 必备环境
* [Opencv4.x](https://opencv.org/releases/)
### 代码使用
colcon build
source install/setup.bash 
ros2 launch target_node launch.py 
### 联系我们
谢炜坡 qq：2113994486

代码还在测试，非最终版本
由于cv::getStructuringElement  (include/target.cpp  void armorDetector::ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold) ) 因为未知原因(无效的内存访问)用不了，导致图像噪音有点严重 ，预测效果不是很理想
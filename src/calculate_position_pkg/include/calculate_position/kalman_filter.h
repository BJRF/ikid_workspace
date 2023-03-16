// #define _MYKALMAN_H
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class MyKalmanFilter
{
    private:
        // 系统模型：x(k+1) = A*x(k) + B*u(k) + w(k)
        float A = 1.0;   // 状态转移矩阵
        float B = 1.0;   // 控制输入矩阵
        float u = 0.0;   // 控制输入
        float w = 0.01;  // 系统噪声方差

        // 观测模型：y(k) = C*x(k) + v(k)
        float C = 1.0;   // 观测矩阵
        float v = 0.1;   // 观测噪声方差

        // 初始状态和协方差矩阵
        float x0 = 0.0;  // 初始状态
        float P0 = 1.0;  // 初始协方差矩阵

        // 定义卡尔曼滤波器
        float x_hat = x0;          // 估计状态
        float P = P0;              // 估计协方差矩阵
        float K;                   // 卡尔曼增益
        float y;                   // 观测值
        float x;                   // 真实状态
        MatrixXd Q = MatrixXd::Identity(1, 1) * w;  // 系统噪声协方差矩阵
        MatrixXd R = MatrixXd::Identity(1, 1) * v;  // 观测噪声协方差矩阵
    public:
        MyKalmanFilter();
        // MyKalmanFilter(double A, double B, double u, double w, double C, double v, double x0, double P0, double x_hat, double P, MatrixXd Q, MatrixXd R){};
        ~MyKalmanFilter();
        float get_my_kalman_filter_result(float x_true);
};



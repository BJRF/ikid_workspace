#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main()
{
    // 系统模型：x(k+1) = A*x(k) + B*u(k) + w(k)
    double A = 1.0;   // 状态转移矩阵
    double B = 1.0;   // 控制输入矩阵
    double u = 0.0;   // 控制输入
    double w = 0.01;  // 系统噪声方差

    // 观测模型：y(k) = C*x(k) + v(k)
    double C = 1.0;   // 观测矩阵
    double v = 0.1;   // 观测噪声方差

    // 初始状态和协方差矩阵
    double x0 = 0.0;  // 初始状态
    double P0 = 1.0;  // 初始协方差矩阵

    // 定义卡尔曼滤波器
    double x_hat = x0;          // 估计状态
    double P = P0;              // 估计协方差矩阵
    double K;                   // 卡尔曼增益
    double y;                   // 观测值
    double x;                   // 真实状态
    MatrixXd Q = MatrixXd::Identity(1, 1) * w;  // 系统噪声协方差矩阵
    MatrixXd R = MatrixXd::Identity(1, 1) * v;  // 观测噪声协方差矩阵

    // 生成一些随机的数据作为真实状态
    VectorXd x_true(100);
    for (int i = 0; i < x_true.size(); i++) {
        x_true(i) = i + (double)rand() / RAND_MAX;
    }

    // 进行卡尔曼滤波
    for (int i = 0; i < x_true.size(); i++) {
        // 系统模型预测
        double x_pred = A * x_hat + B * u;
        double P_pred = A * P * A + Q(0, 0);

        // 获取观测值
        y = C * x_true(i) + v * ((double)rand() / RAND_MAX - 0.5);

        // 卡尔曼增益计算
        K = P_pred * C / (C * P_pred * C + R(0, 0));

        // 更新估计状态和协方差矩阵
        x_hat = x_pred + K * (y - C * x_pred);
        P = (1 - K * C) * P_pred;

        // 输出结果
        std::cout << "x_true = " << x_true(i) << ", x_hat = " << x_hat << std::endl;
    }

    return 0;
}

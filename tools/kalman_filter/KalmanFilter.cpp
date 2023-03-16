#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(int stateSize_ = 0, int measSize_ = 0, int uSize_=0) :stateSize(stateSize_), measSize(measSize_), uSize(uSize_)
{
    if (stateSize == 0 || measSize == 0)
    {
        std::cout << "Error, State size and measurement size must bigger than 0\n";
    }

    x.resize(stateSize);
    x.setZero();

    A.resize(stateSize, stateSize);
    A.setIdentity();

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measSize, stateSize);
    H.setZero();

    z.resize(measSize);
    z.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measSize, measSize);
    R.setZero();
}

void KalmanFilter::init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_)
{
    x = x_;
    P = P_;
    R = R_;
    Q = Q_;
}
Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_)
{
    A = A_;
    B = B_;
    u = u_;
    x = A*x + B*u;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q;
    return x;
}

Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_)
{
    A = A_;
    x = A*x;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q; 
//  cout << "P-=" << P<< endl;
    return x;
}

void KalmanFilter::update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas)
{
    H = H_;
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
    temp1 = H*P*Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P*Ht*temp2;
    z = H*x;
    x = x + K*(z_meas-z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K*H)*P;
//  cout << "P=" << P << endl;
}

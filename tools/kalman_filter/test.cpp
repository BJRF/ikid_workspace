#include <iostream>
#include "Eigen/Dense"
#include "KalmanFilter.h"
#include <fstream>

using namespace std;
#define N 1000
#define T 0.01

double func(double& x);
double data_x[N],data_y[N];

double func(double& x)
{
    double res = 5*x*x;
    return res;
}

float sample(float x0, float v0, float acc, float t)
{
    return x0 + v0*t + 1 / 2 * acc*t*t;
}

float GetRand()
{
    return 0.5 * rand() / RAND_MAX - 0.25;
}

int main()
{
    ofstream fout;
    fout.open("data.txt");
    float t;
    for (int i = 0; i < N; i++)
    {
        /*data_x[i] = i*T*10;
        data_y[i] = func(data_x[i]);*/
        t = i*T;
        data_x[i] = sample(0, -4, 0, t) + GetRand();
        data_y[i] = sample(0, 6.5, 0, t) + GetRand();
    }
    int stateSize = 6;
    int measSize = 2;
    int controlSize = 0;
    KalmanFilter kf(stateSize, measSize, controlSize);
    Eigen::MatrixXd A(stateSize, stateSize);
    A << 1, 0, T, 0, 1 / 2 * T*T, 0,
        0, 1, 0, T, 0, 1 / 2 * T*T,
        0, 0, 1, 0, T, 0, 
        0, 0, 0, 1, 0, T,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    //cout << A;
    Eigen::MatrixXd B(0,0);
    Eigen::MatrixXd H(measSize, stateSize);
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;
    //cout << H;
    Eigen::MatrixXd P(stateSize, stateSize);
    P.setIdentity();
    Eigen::MatrixXd R(measSize, measSize);
    R.setIdentity()*0.01;
    Eigen::MatrixXd Q(stateSize, stateSize);
    Q.setIdentity()*0.001;
    Eigen::VectorXd x(stateSize);
    Eigen::VectorXd u(0);
    Eigen::VectorXd z(measSize);
    z.setZero();
    Eigen::VectorXd res(stateSize);

    for (int i = 0; i < N; i++)
    {
        //cout << "state_" << i << ":\n";
        if (i == 0)
        {
            x << data_x[i], data_y[i], 0, 0, 0, 0;
            kf.init(x, P, R, Q);
        }
        res<< kf.predict(A);
        z << data_x[i], data_y[i];
        kf.update(H,z);
        fout << data_x[i] << " " << res[0] << " " << data_y[i] << " " << res[1] << " " << res[2] << " " << res[3] << " " << res[4] << " " << res[5] << endl;
    }
    fout.close();
    cout << "Done, use python script to draw the figure....\n";
    return 0;
}
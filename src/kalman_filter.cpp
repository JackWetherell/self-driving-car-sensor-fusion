#include "kalman_filter.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter(){}


KalmanFilter::~KalmanFilter(){}


void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}


// Implement predict formula
void KalmanFilter::Predict()
{
    this->x_ = F_ * x_;
    this->P_ = F_ * P_ * F_.transpose() + Q_;
}


// Implement update formula
void KalmanFilter::Update(const VectorXd &z)
{
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    VectorXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd S_inverse = S.inverse();
    MatrixXd K =  P_ * H_.transpose() * S_inverse;
    this->x_ = x_ + (K * y);
    this->P_ = (I - K * H_) * P_;
}


// Implement extended KF
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
    double rho = sqrt(std::pow(px,2) + std::pow(py,2));
    double theta = std::atan2(py, px);
    double rhod = (px*vx + py*vy) / rho;
    VectorXd h = VectorXd(3);
    h << rho, theta, rhod;
    VectorXd y = z - h;
    while (y(1) > M_PI || y(1) < -M_PI)
    {
        if (y(1) > M_PI)
        {
            y(1) -= M_PI;
        }
        else
        {
            y(1) += M_PI;
        }
    }
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd S_inverse = S.inverse();
    MatrixXd K =  P_ * H_.transpose() * S_inverse;
    this->x_ = x_ + (K * y);
    this->P_ = (I - K * H_) * P_;
}

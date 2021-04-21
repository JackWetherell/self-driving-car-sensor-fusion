#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"


class KalmanFilter
{
    public:
        KalmanFilter();
        virtual ~KalmanFilter();
        /**
        * Init Initializes Kalman filter
        * @param x_in Initial state
        * @param P_in Initial state covariance
        * @param F_in Transition matrix
        * @param H_in Measurement matrix
        * @param R_in Measurement covariance matrix
        * @param Q_in Process covariance matrix
        */
        void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);
        void Predict();
        void Update(const Eigen::VectorXd &z);
        void UpdateEKF(const Eigen::VectorXd &z);
        Eigen::VectorXd x_; // state vector
        Eigen::MatrixXd P_; // state covariance matrix
        Eigen::MatrixXd F_; // State transition matrix
        Eigen::MatrixXd Q_; // Process covariance matrix
        Eigen::MatrixXd H_; // Measurement matrix
        Eigen::MatrixXd R_; // Measurement covariance matrix

};


#endif // KALMAN_FILTER_H_

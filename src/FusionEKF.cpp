#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


// Constructor
FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // Initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    // Measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;


    // Measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // Initializing P
    this->ekf_.P_ = MatrixXd(4, 4);
    this->ekf_.P_ << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1000, 0,
                     0, 0, 0, 1000;
    H_laser_ << 1, 0, 0, 0,   // Only mesures position, zeros for velocity
                0, 1, 0, 0;
}


// Destructor
FusionEKF::~FusionEKF(){}


// Process measurement function
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_)
    {
        // First measurement
        this->ekf_.x_ = VectorXd(4);
        this->ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double rhod = measurement_pack.raw_measurements_[2];
            double x = rho*std::cos(phi);
            double y = rho * std::sin(phi);
            double vx = rhod * std::cos(phi);
            double vy = rhod * std::sin(phi);
            this->ekf_.x_ << x, y, vx, vy;
        }

        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            this->ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        }

        // Saving first timestamp in seconds
        this->previous_timestamp_ = measurement_pack.timestamp_;


        // Done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    // Pediction
    double dt = (measurement_pack.timestamp_ - this->previous_timestamp_) / std::pow(10.0, 6); // from timestamp to real time
    this->previous_timestamp_ = measurement_pack.timestamp_;

    // Pre compute the powers for efficiency
    double dt2 = std::pow(dt, 2);
    double dt3 = std::pow(dt, 3);
    double dt4 = std::pow(dt, 4);

    // State transition matrix update
    this->ekf_.F_ = MatrixXd(4, 4);
    this->ekf_.F_ << 1, 0, dt, 0,
                     0, 1, 0, dt,
                     0, 0, 1, 0,
                     0, 0, 0, 1;

    // Noise values
    double noise_ax = 9.0;
    double noise_ay = 9.0;

    // Q as defined in notes
    this->ekf_.Q_ = MatrixXd(4, 4);
    this->ekf_.Q_ << dt4/4 * noise_ax, 0, dt3/2 * noise_ax, 0,
	                   0, dt4/4 * noise_ay, 0, dt3/2 * noise_ay,
	                   dt3/2 * noise_ax, 0, dt2 * noise_ax, 0,
 	                   0, dt3/2 * noise_ay, 0, dt2 * noise_ay;
    this->ekf_.Predict();

    // Radar update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        this->ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        this->ekf_.R_ = R_radar_;
        this->ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    // Lidar update
    else
    {

        this->ekf_.H_ = H_laser_;
        this->ekf_.R_ = R_laser_;
        this->ekf_.Update(measurement_pack.raw_measurements_);
    }
}

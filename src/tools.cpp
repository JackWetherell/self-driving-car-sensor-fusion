#include "tools.h"
#include <iostream>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools(){}
Tools::~Tools(){}


// Function to compure the root mean square error of the estimations given a ground truth
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
    int count = estimations.size(); // Get the data count
    VectorXd square_error(4);
    VectorXd mean_square_error(4);
    VectorXd root_mean_square_error(4);
    square_error << 0,0,0,0; // Initialize the vector
    for(int i=0; i < estimations.size(); i++)
    {
        VectorXd difference = estimations[i] - ground_truth[i];
        VectorXd difference_square = difference.array() * difference.array();
        square_error += difference_square;
    }
    mean_square_error = square_error / count; // Calculate the mean
    root_mean_square_error = mean_square_error.array().sqrt(); // Square root
    return root_mean_square_error;
}


// Function to construct the Jacobian matrix of the state
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    // Define matrix object
    MatrixXd jacobian(3,4);

    // Define each component of the state
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // Define constants from formula
    double a = std::pow(px,2) + std::pow(py,2);
    double b = std::sqrt(a);
    double c = (a*b);

    // Fill in the elements of the matrix
    jacobian << (px/b), (py/b), 0, 0,
                -(py/a), (px/a), 0, 0,
                py*(vx*py-vy*px)/c, px*(px*vy-py*vx)/c, px/b, py/b;

    return jacobian;
}

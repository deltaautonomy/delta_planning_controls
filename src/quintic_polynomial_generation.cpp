/*=================================================================
 *
 * Delta Vehicle Control.cpp
 * author: Prateek Parmeshwar
 *
 *=================================================================*/
#include <delta_planning_controls/quintic_polynomial_generation.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

QuinticPolynomialGeneration::QuinticPolynomialGeneration()
    : m_ctrl_freq(0.1)
    , m_planning_time(1)
{
}

QuinticPolynomialGeneration::QuinticPolynomialGeneration(double ctrl_freq, double max_acc_x, min_acc_x)
{
    m_ctrl_freq = ctrl_freq;
    // Change 3
    m_max_acceleration_x = max_acc_x;
    m_min_acceleration_x = min_acc_x;
}
// Define member functions
double QuinticPolynomialGeneration::getCtrlFreq() { return m_ctrl_freq; } // Getter for control freq

void QuinticPolynomialGeneration::setCtrlFreq(double new_ctrl_freq) { m_ctrl_freq = new_ctrl_freq; } // Setter for control freq

double QuinticPolynomialGeneration::getMaxPlanningTime(double curr_velocity_x)
{
    double planning_time = curr_velocity_x/m_min_acceleration_x; // v=a*t a--> max deceleration
    return planning_time;
}

double 
// Member function for generating evasive trajectory
MatrixXd QuinticPolynomialGeneration::getPolynomialCoefficients(vector<double> boundary_vals)
{
    // Boundary vals is [xi,yi,xf,yf,vxi,vyi,axi,ayi]
    // minimum jerk trajectory is a 5th order polynomial
    // y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // Given initial and final values in pos, vel and acc (Note final acc is 0 and final vel is 0) coeffs are:
    double T = m_planning_time;
    MatrixXd coeffs(4, 6); // Matrix of coefficients
    // Populate matrix with coeffs for x in pos and vel
    // Position in x
    coeffs(0, 0) = boundary_vals[0]; // a0x = xi
    coeffs(0, 1) = boundary_vals[4]; // a1x = vxi
    coeffs(0, 2) = boundary_vals[6] / 2; // a2x = axi/2
    // a3x = -(20*xi - 20*xf + 12*T*vxi + 11*axi*T**2 - 8*axi*T)/(2*T**3)
    coeffs(0, 3) = -(20 * boundary_vals[0] - 20 * boundary_vals[2] + 12 * T * boundary_vals[4] + 11 * boundary_vals[6] * T * T - 8 * boundary_vals[6] * T) / (2 * pow(T, 3));
    // a4x = (30*xi - 30*xf + 16*T*vxi + 17*axi*T**2 - 14*axi*T)/(2*T**4)
    coeffs(0, 4) = (30 * boundary_vals[0] - 30 * boundary_vals[2] + 16 * T * boundary_vals[4] + 17 * boundary_vals[6] * T * T - 14 * boundary_vals[6] * T) / (2 * pow(T, 4));
    // a5x = -(12*xi - 12*xf + 6*T*vxi + 7*axi*T**2 - 6*axi*T)/(2*T**5)
    coeffs(0, 5) = -(12 * boundary_vals[0] - 12 * boundary_vals[2] + 6 * T * boundary_vals[4] + 7 * boundary_vals[6] * T * T - 6 * boundary_vals[6] * T) / (2 * pow(T, 5));
    // Position in y
    // Position in x
    coeffs(1, 0) = boundary_vals[1]; // a0y = yi
    coeffs(1, 1) = boundary_vals[5]; // a1y = vyi
    coeffs(1, 2) = boundary_vals[7] / 2; // a2y = ayi/2
    // a3y = -(20*yi - 20*yf + 12*T*vyi + 11*ayi*T**2 - 8*ayi*T)/(2*T**3)
    coeffs(1, 3) = -(20 * boundary_vals[1] - 20 * boundary_vals[3] + 12 * T * boundary_vals[5] + 11 * boundary_vals[7] * T * T - 8 * boundary_vals[7] * T) / (2 * pow(T, 3));
    // a4y = (30*yi - 30*yf + 16*T*vyi + 17*ayi*T**2 - 14*ayi*T)/(2*T**4)
    coeffs(1, 4) = (30 * boundary_vals[1] - 30 * boundary_vals[3] + 16 * T * boundary_vals[5] + 17 * boundary_vals[7] * T * T - 14 * boundary_vals[7] * T) / (2 * pow(T, 4));
    // a5y = -(12*yi - 12*yf + 6*T*vyi + 7*ayi*T**2 - 6*ayi*T)/(2*T**5)
    coeffs(1, 5) = -(12 * boundary_vals[1] - 12 * boundary_vals[3] + 6 * T * boundary_vals[5] + 7 * boundary_vals[7] * T * T - 6 * boundary_vals[7] * T) / (2 * pow(T, 5));

    // Velocity in x
    coeffs(2, 0) = coeffs(0, 1);
    coeffs(2, 1) = 2 * coeffs(0, 2);
    coeffs(2, 2) = 3 * coeffs(0, 3);
    coeffs(2, 3) = 4 * coeffs(0, 4);
    coeffs(2, 4) = 5 * coeffs(0, 5);
    coeffs(2, 5) = 0;

    // Velocity in y
    coeffs(3, 0) = coeffs(1, 1);
    coeffs(3, 1) = 2 * coeffs(1, 2);
    coeffs(3, 2) = 3 * coeffs(1, 3);
    coeffs(3, 3) = 4 * coeffs(1, 4);
    coeffs(3, 4) = 5 * coeffs(1, 5);
    coeffs(3, 5) = 0;

    return coeffs;
}

MatrixXd QuinticPolynomialGeneration::getEvasiveTrajectory(vector<double> boundary_vals)
{
    MatrixXd coeffs = getPolynomialCoefficients(boundary_vals);

    double dt = 1 / m_ctrl_freq;
    int num_steps = (int)m_planning_time * m_ctrl_freq;

    MatrixXd reference_trajectory(num_steps, 4);
    for (int i = 0; i < num_steps; i++) {
        VectorXd time_step(6, 1);
        time_step << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5); // Define time vector
        VectorXd tmp = coeffs * time_step; // Get traj info x,y vx, vy
        reference_trajectory.row(i) = tmp.transpose();
    }
    return reference_trajectory;
}

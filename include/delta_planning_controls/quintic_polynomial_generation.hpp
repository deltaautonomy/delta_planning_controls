/*=================================================================
 *
 * Delta Vehicle Control.h
 * author: Prateek Parmeshwar
 *
 *=================================================================*/
#ifndef DELTA_VEHICLE_CONTROL_H
#define DELTA_VEHICLE_CONTROL_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class QuinticPolynomialGeneration {
    // Define member variables
private:
    double m_ctrl_freq; // Controller frequency
    // Change 2
    double m_max_acceleration_x; // Max longitudinal acceleration
    double m_min_acceleration_x; // Max longitudinal deceleration 

public:
    QuinticPolynomialGeneration();

    QuinticPolynomialGeneration(double ctrl_freq, double planning_time);

    // Define member functions
    double getCtrlFreq(); // Getter for control freq
    double getMaxPlanningTime(); // Getter for planning time
    double getFinalPoseX(); // Get final x position

    void setCtrlFreq(double new_ctrl_freq); // Setter for control freq

    // Member function for getting coefficients of 5th order polynomial
    MatrixXd getPolynomialCoefficients();

    // Member function to generate evasive trajectory
    MatrixXd getEvasiveTrajectory();
};

#endif

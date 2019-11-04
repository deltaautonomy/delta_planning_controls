/*=================================================================
 *
 * Delta Vehicle Control.h
 * author: Prateek Parmeshwar
 *
 *=================================================================*/
#ifndef QUINTIC_POLYNOMIAL_GENERATION_H
#define QUINTIC_POLYNOMIAL_GENERATION_H

#include <eigen3/Eigen/Dense>
#include "delta_planning_controls/vehicle_state.hpp"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class QuinticPolynomialGeneration {
    // Define member variables
private:
    double m_ctrl_freq; // Controller frequency
    double m_max_acceleration_x; // Max longitudinal acceleration
    double m_min_acceleration_x; // Max longitudinal deceleration 

public:
    QuinticPolynomialGeneration();

    QuinticPolynomialGeneration(double ctrl_freq, double max_acc_x, double min_acc_x);

    // Define member functions
    double getCtrlFreq(); // Getter for control freq
    double getMaxPlanningTime(VehicleState _ego_state); // Getter for planning time
    double getFinalPoseX(VehicleState _ego_state); // Get final x position
    MatrixXd homogenousTransWorldEgo(VehicleState _ego_state); // Get homogenous transform of ego vehicle wrt world

    void setCtrlFreq(double new_ctrl_freq); // Setter for control freq

    // Member function for getting coefficients of 5th order polynomial
    MatrixXd getPolynomialCoefficients(VehicleState _ego_state);

    // Member function to generate evasive trajectory
    MatrixXd getEvasiveTrajectory(VehicleState _ego_state);
};

#endif

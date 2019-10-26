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
    double m_planning_time; // Time in which the trajectory is executed

public:
    QuinticPolynomialGeneration();

    QuinticPolynomialGeneration(double ctrl_freq, double planning_time);

    // Define member functions
    double getCtrlFreq(); // Getter for control freq
    double getPlanningTime(); // Getter for planning time

    void setCtrlFreq(double new_ctrl_freq); // Setter for control freq
    void setPlanningTime(double new_planning_time); //Setter for planning time

    // Member function for getting coefficients of 5th order polynomial
    MatrixXd getPolynomialCoefficients(vector<double> boundary_vals);

    // Member function to generate evasive trajectory
    MatrixXd getEvasiveTrajectory(vector<double> boundary_vals);
};

#endif

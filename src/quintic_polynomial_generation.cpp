/*=================================================================
 *
 * Delta Vehicle Control.cpp
 * author: Prateek Parmeshwar
 *
 *=================================================================*/
#include "delta_planning_controls/quintic_polynomial_generation.hpp"
#include "delta_planning_controls/vehicle_state.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;

QuinticPolynomialGeneration::QuinticPolynomialGeneration()
    : m_ctrl_freq(0.1)
    , m_max_acceleration_x(3.0)
    , m_min_acceleration_x(8.0)
    , m_shoulder_const(-5.0)
{
}

QuinticPolynomialGeneration::QuinticPolynomialGeneration(double ctrl_freq, double max_acc_x, double min_acc_x, double shoulder_const)
{
    m_ctrl_freq = ctrl_freq;
    m_max_acceleration_x = max_acc_x;
    m_min_acceleration_x = min_acc_x;
    m_shoulder_const = shoulder_const;
}
// Define member functions
double QuinticPolynomialGeneration::getCtrlFreq() { return m_ctrl_freq; } // Getter for control freq

void QuinticPolynomialGeneration::setCtrlFreq(double new_ctrl_freq) { m_ctrl_freq = new_ctrl_freq; } // Setter for control freq

double QuinticPolynomialGeneration::getMaxPlanningTime(VehicleState _ego_state)
{
    double planning_time = _ego_state.vx/m_min_acceleration_x; // v=a*t a--> max deceleration
    return planning_time;
}

double QuinticPolynomialGeneration::getFinalPoseX(VehicleState _ego_state)
{
    double t = getMaxPlanningTime(_ego_state);
    double xf = _ego_state.vx*t - 0.5*m_min_acceleration_x*t*t;
    // cout<<"Planning Time: "<<t<<'\n';

    // cout<<"Vel x value: "<<_ego_state.vx<<'\n';
    // cout<<"Final x value: "<<xf<<'\n';
    return xf;
}

MatrixXd QuinticPolynomialGeneration::homogenousTransWorldEgo(VehicleState _ego_state)
{
    MatrixXd homo_trans(3,3);
    homo_trans<<cos(_ego_state.yaw), -sin(_ego_state.yaw), _ego_state.x,
                sin(_ego_state.yaw), cos(_ego_state.yaw), _ego_state.y,
                0, 0, 1;
    return homo_trans;
}

MatrixXd QuinticPolynomialGeneration::getBoundaryValsWorldFrame(VehicleState _ego_state)
{
    MatrixXd boundary_vals(4,2);
    MatrixXd homo_trans = homogenousTransWorldEgo(_ego_state);   

    Vector3d ego_pose_fin(getFinalPoseX(_ego_state), m_shoulder_const, 1); // pose in ego frame
    Vector3d ego_vel(_ego_state.vx, _ego_state.vy, 0); // velocity in ego frame
    Vector3d ego_acc(_ego_state.acc_x, _ego_state.acc_y, 0); // acceleration in ego frame

    Vector3d world_pose_fin = homo_trans*ego_pose_fin;
    Vector3d world_vel = homo_trans*ego_vel;
    Vector3d world_acc = homo_trans*ego_acc;

    boundary_vals.row(0)<<_ego_state.x, _ego_state.y;
    boundary_vals.row(1)<<world_pose_fin(0),world_pose_fin(1);
    boundary_vals.row(2)<<world_vel(0),world_vel(1);
    boundary_vals.row(3)<<world_acc(0),world_acc(1);

    // cout<<boundary_vals<<'\n'<<endl;
    return boundary_vals; 

}
// Member function for generating evasive trajectory
MatrixXd QuinticPolynomialGeneration::getPolynomialCoefficients(VehicleState _ego_state)
{
    // Boundary vals is [xi,yi,xf,yf,vxi,vyi,axi,ayi]
    // minimum jerk trajectory is a 5th order polynomial
    // y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // Given initial and final values in pos, vel and acc (Note final acc is 0 and final vel is 0) coeffs are:
    MatrixXd boundary_vals = getBoundaryValsWorldFrame(_ego_state);
    // Get vals in world frame
    // cout<<
    double xi = boundary_vals(0,0);
    double yi = boundary_vals(0,1);
    // cout<<"Xi:"<<xi<<" Yi: "<<yi<<'\n'<<endl;

    double xf = boundary_vals(1,0);
    double yf = boundary_vals(1,1);
    // cout<<"World Pose: "<<world_pose_fin<<'\n'<<endl;
    double vxi = boundary_vals(2,0);
    double vyi = boundary_vals(2,1);
    // cout<<"World Vel: "<<world_vel<<'\n'<<endl;
    // cout<<"World Acc: "<<world_acc<<'\n'<<endl;
    // cout<<"vxi: "<<_ego_state.vx<<'\n'<<endl;

    double axi = boundary_vals(3,0);
    double ayi = boundary_vals(3,1);

    double T = getMaxPlanningTime(_ego_state);
    MatrixXd coeffs(4, 6); // Matrix of coefficients
    // Populate matrix with coeffs for x in pos and vel
    // Position in x
    coeffs(0, 0) = xi; // a0x = xi
    coeffs(0, 1) = vxi; // a1x = vxi
    coeffs(0, 2) = axi/2; // a2x = axi/2
    // a3x = -(20*xi - 20*xf + 12*T*vxi + 11*axi*T**2 - 8*axi*T)/(2*T**3)
    coeffs(0, 3) = -(20 * xi - 20 * xf + 12 * T * vxi + 3 * axi * T * T) / (2 * pow(T, 3));
    // a4x = (30*xi - 30*xf + 16*T*vxi + 17*axi*T**2 - 14*axi*T)/(2*T**4)
    coeffs(0, 4) = (30 * xi - 30 * xf + 16 * T * vxi + 3 * axi * T * T) / (2 * pow(T, 4));
    // a5x = -(12*xi - 12*xf + 6*T*vxi + 7*axi*T**2 - 6*axi*T)/(2*T**5)
    coeffs(0, 5) = -(12 * xi - 12 * xf + 6 * T * vxi + axi * T * T) / (2 * pow(T, 5));
    
    
    // Position in y
    coeffs(1, 0) = yi; // a0y = yi
    coeffs(1, 1) = vyi; // a1y = vyi
    coeffs(1, 2) = ayi/2; // a2y = ayi/2
    // a3y = -(20*yi - 20*yf + 12*T*vyi + 11*ayi*T**2 - 8*ayi*T)/(2*T**3)
    coeffs(1, 3) = -(20 * yi - 20 * yf + 12 * T * vyi + 3 * ayi * T * T) / (2 * pow(T, 3));
    // a4y = (30*yi - 30*yf + 16*T*vyi + 17*ayi*T**2 - 14*ayi*T)/(2*T**4)
    coeffs(1, 4) = (30 * yi - 30 * yf + 16 * T * vyi + 3 * ayi * T * T) / (2 * pow(T, 4));
    // a5y = -(12*yi - 12*yf + 6*T*vyi + 7*ayi*T**2 - 6*ayi*T)/(2*T**5)
    coeffs(1, 5) = -(12 * yi - 12 * yf + 6 * T * vyi + ayi * T * T) / (2 * pow(T, 5));

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

MatrixXd QuinticPolynomialGeneration::getEvasiveTrajectory(VehicleState _ego_state)
{
    MatrixXd coeffs = getPolynomialCoefficients(_ego_state);

    double dt = 1 / m_ctrl_freq;
    int num_steps = (int)(getMaxPlanningTime(_ego_state) * m_ctrl_freq);
    MatrixXd reference_trajectory(num_steps, 4);
    for (int i = 0; i < num_steps; i++) {
        VectorXd time_step(6, 1);
        time_step << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5); // Define time vector
        VectorXd tmp = coeffs * time_step; // Get traj info x,y vx, vy
        reference_trajectory.row(i) = tmp.transpose();
        dt = dt + 1/m_ctrl_freq;
    }
    MatrixXd world_vel(3,num_steps);
    for (int j=0; j<num_steps; j++)
    {
        world_vel.col(j)<<reference_trajectory(j,2),reference_trajectory(j,3),0;
    }

    MatrixXd homo_trans = homogenousTransWorldEgo(_ego_state);
    // cout<<"world vel: "<<homo_trans.inverse()<<'\n'<<endl;
    MatrixXd ego_vel = homo_trans.inverse()*world_vel;
    for (int k=0;k<num_steps;k++)
    {
        reference_trajectory(k,2) = ego_vel(0,k);
        reference_trajectory(k,3) = ego_vel(1,k);
    }
    // cout<<"traj: "<<reference_trajectory<<'\n'<<endl;
    return reference_trajectory;
}

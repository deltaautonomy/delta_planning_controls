
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

double QuinticPolynomialGeneration::getFinalPoseX(VehicleState _ego_state, double planning_time)
{
    double t = planning_time;
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

MatrixXd QuinticPolynomialGeneration::getBoundaryValsWorldFrame(VehicleState _ego_state, VehicleState final_state)
{
    MatrixXd boundary_vals(6,2);
    MatrixXd homo_trans = homogenousTransWorldEgo(_ego_state);   

    // Ego init pose already in world frame
    Vector3d ego_vel_init(_ego_state.vx, _ego_state.vy, 0); // velocity in ego frame
    Vector3d ego_acc_init(_ego_state.acc_x, _ego_state.acc_y, 0); // acceleration in ego frame

    Vector3d ego_pose_fin(final_state.x, final_state.y, 1); // pose in ego frame final
    Vector3d ego_vel_fin(final_state.vx, final_state.vy, 0); // vel in ego frame final
    Vector3d ego_acc_fin(final_state.acc_x, final_state.acc_y, 0); // acc in ego frame final

    // Transform these values in world frame
    Vector3d world_vel_init = homo_trans*ego_vel_init;
    Vector3d world_acc_init = homo_trans*ego_acc_init;

    Vector3d world_pose_fin = homo_trans*ego_pose_fin;
    Vector3d world_vel_fin = homo_trans*ego_vel_fin;
    Vector3d world_acc_fin = homo_trans*ego_acc_fin;

    boundary_vals.row(0)<<_ego_state.x, _ego_state.y;
    boundary_vals.row(1)<<world_vel_init(0),world_vel_init(1);
    boundary_vals.row(2)<<world_acc_init(0),world_acc_init(1);

    boundary_vals.row(3)<<world_pose_fin(0),world_pose_fin(1);
    boundary_vals.row(4)<<world_vel_fin(0),world_vel_fin(1);
    boundary_vals.row(5)<<world_acc_fin(0),world_acc_fin(1);

    // cout<<boundary_vals<<'\n'<<endl;
    return boundary_vals; 

}
// Member function for generating evasive trajectory
MatrixXd QuinticPolynomialGeneration::getPolynomialCoefficients(VehicleState _ego_state, VehicleState final_state, double planning_time)
{
    // Boundary vals is [xi,yi,xf,yf,vxi,vyi,axi,ayi]
    // minimum jerk trajectory is a 5th order polynomial
    // y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // Given initial and final values in pos, vel and acc (Note final acc is 0 and final vel is 0) coeffs are:
    MatrixXd boundary_vals = getBoundaryValsWorldFrame(_ego_state, final_state);
    // Get vals in world frame

    double T = planning_time;

    // Initial vals
    double xi  = boundary_vals(0,0);
    double yi = boundary_vals(0,1);
    double vxi  = boundary_vals(1,0);
    double vyi = boundary_vals(1,1);
    double axi  = boundary_vals(2,0);
    double ayi = boundary_vals(2,1);

    double xf = boundary_vals(3,0);
    double yf = boundary_vals(3,1);
    double vxf = boundary_vals(4,0);
    double vyf = boundary_vals(4,1);
    double axf = boundary_vals(5,0);
    double ayf = boundary_vals(5,1);

    MatrixXd coeffs(4, 6); // Matrix of coefficients
    // Populate matrix with coeffs for x in pos and vel

    // a0 = xi
    // a1 = vi
    // a2 = Ai/2
    // a3 = -(20*xi-20*xf + 8*T*vf + 12*T*vi -Af*T^2 + 3*Ai*T^2)/(2*T^3)
    // a4 = (30*xi- 30*xf + 14*T*vf + 16*T*vi - 2*Af*T^2 + 3*Ai*T^2)/(2*T^4)
    // a5 = -(12*xi-12*xf + 6*T*vf + 6*T*vi - Af*T^2 + Ai*T^2)/(2*T^5)

     // Pose x
    coeffs(0,0) = xi;
    coeffs(0,1) = vxi;
    coeffs(0,2) = axi/2;
    coeffs(0,3) = -(20*xi - 20*xf + 8*T*vxf + 12*T*vxi - axf*T*T + 3*axi*T*T)/(2*pow(T,3));
    coeffs(0,4) = (30*xi- 30*xf + 14*T*vxf + 16*T*vxi - 2*axf*T*T + 3*axi*T*T)/(2*pow(T,4));
    coeffs(0,5) = -(12*xi-12*xf + 6*T*vxf + 6*T*vxi - axf*T*T + axi*T*T)/(2*pow(T,5));
    
    // Pose y
    coeffs(1,0) = yi;
    coeffs(1,1) = vyi;
    coeffs(1,2) = ayi/2;
    coeffs(1,3) = -(20*yi - 20*yf + 8*T*vyf + 12*T*vyi - ayf*T*T + 3*ayi*T*T)/(2*pow(T,3));
    coeffs(1,4) = (30*yi- 30*yf + 14*T*vyf + 16*T*vyi - 2*ayf*T*T + 3*ayi*T*T)/(2*pow(T,4));
    coeffs(1,5) = -(12*yi-12*yf + 6*T*vyf + 6*T*vyi - ayf*T*T + ayi*T*T)/(2*pow(T,5));

    // velocity in x
    coeffs(2, 0) = coeffs(0, 1);
    coeffs(2, 1) = 2 * coeffs(0, 2);
    coeffs(2, 2) = 3 * coeffs(0, 3);
    coeffs(2, 3) = 4 * coeffs(0, 4);
    coeffs(2, 4) = 5 * coeffs(0, 5);
    coeffs(2, 5) = 0;

    // velocity in y
    coeffs(3, 0) = coeffs(1, 1);
    coeffs(3, 1) = 2 * coeffs(1, 2);
    coeffs(3, 2) = 3 * coeffs(1, 3);
    coeffs(3, 3) = 4 * coeffs(1, 4);
    coeffs(3, 4) = 5 * coeffs(1, 5);
    coeffs(3, 5) = 0;


    return coeffs;
}

MatrixXd QuinticPolynomialGeneration::getEvasiveTrajectory(VehicleState _ego_state, VehicleState final_state, double planning_time)
{
    MatrixXd coeffs = getPolynomialCoefficients(_ego_state,final_state,planning_time);

    double dt = 1 / m_ctrl_freq;
    int num_steps = (int)(planning_time * m_ctrl_freq);
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

MatrixXd QuinticPolynomialGeneration::getManeuver(VehicleState _ego_state, double lane_val, int type_maneuver)
{
    // Get evasive maneuver
    // 1 --> CAS original maneuver
    // 2 --> Brake
    // 3 --> Sharp maneuver
    
    // VehicleState(double inp_x, double inp_y, double inp_yaw, double inp_vx, double inp_vy, double inp_yaw_rate, 
    //    double inp_acc_x, double inp_acc_y))
    if (type_maneuver == 1)
    {
        double planning_time = getMaxPlanningTime(_ego_state);
        VehicleState final_state(getFinalPoseX(_ego_state, planning_time), lane_val,0,0,0,0,0,0);
        return getEvasiveTrajectory(_ego_state, final_state, planning_time);
    }

    if (type_maneuver == 2)
    {
        double planning_time = getMaxPlanningTime(_ego_state);
        VehicleState final_state(getFinalPoseX(_ego_state, planning_time),0,0,0,0,0,0,0);
        return getEvasiveTrajectory(_ego_state, final_state, planning_time);
    }

    if (type_maneuver == 3)
    {
        // Solve two trajectory optimizations

        // Plan first trajectory
        lane_val = 2*lane_val;
        double planning_time = getMaxPlanningTime(_ego_state);
        double alpha = 0.4;
        double xf_1 = alpha*getFinalPoseX(_ego_state, planning_time);
        // cout<<"xf "<<xf_1<<endl;
        double vf_1 = sqrt(abs(pow(_ego_state.vx,2) - 2*m_min_acceleration_x*xf_1)); // v^2 - u^2 = 2*a*s
        double time_traj_1 = abs(vf_1 - _ego_state.vx)/m_min_acceleration_x;
        
        VehicleState mid_state(xf_1, lane_val/2, 0, vf_1, alpha*lane_val/(time_traj_1), 0, m_min_acceleration_x, alpha*lane_val/(time_traj_1*time_traj_1)); // All in ego frame
        // VehicleState mid_state(xf_1, lane_val, 0, vf_1, 0, 0, 0, 0);

        MatrixXd traj_1 = getEvasiveTrajectory(_ego_state, mid_state, time_traj_1);
        // return traj_1;

        //Plan second trajectory
        double time_traj_2 = vf_1/m_min_acceleration_x;
        // Now mid_state pose should be in world frame
        Vector3d mid_init(mid_state.x, mid_state.y, 1);
        Vector3d world_mid_init = homogenousTransWorldEgo(_ego_state)*mid_init;
        mid_state.x = world_mid_init(0);
        mid_state.y = world_mid_init(1);
        double xf_2 = getFinalPoseX(mid_state, time_traj_2);
        VehicleState final_state(xf_2, (1-alpha)*lane_val, 0, 0, 0, 0, 0, 0);
        MatrixXd traj_2 = getEvasiveTrajectory(mid_state, final_state, time_traj_2);

        MatrixXd final_trajectory(traj_1.rows()+traj_2.rows(), 4);
        final_trajectory<<traj_1,traj_2;
        // cout<<"TRAJ:::::::   "<<final_trajectory<<'\n\n\n\n';
        return final_trajectory;

    }
}

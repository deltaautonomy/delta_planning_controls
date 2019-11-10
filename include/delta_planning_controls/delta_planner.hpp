#ifndef _DELTA_PLANNER_H_
#define _DELTA_PLANNER_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <delta_msgs/EgoStateEstimate.h>
#include <delta_msgs/LaneMarkingArray.h>
#include <delta_planning_controls/utils.hpp>
#include <delta_planning_controls/vehicle_state.hpp>
#include <delta_planning_controls/pid_controller.hpp>
#include <carla_ros_bridge_msgs/CarlaEgoVehicleControl.h>
// #include <delta_planning_controls/PIDReconfigureConfig.h>
#include <delta_planning_controls/quintic_polynomial_generation.hpp>

class DeltaPlanner {

private:
    ros::NodeHandle _private_nh;

    // Constants
    double _dt;
    double _steer_max;
    double _steer_min;
    double _brake_max;
    double _brake_min;
    double _throttle_max;
    double _throttle_min;
    double _speed_kp;
    double _speed_ki;
    double _speed_kd;
    double _steer_k1;
    double _steer_k2;

    double _planner_freq;
    double _max_acceleration_x;
    double _min_acceleration_x;
    double _max_acceleration_y;
    double _min_acceleration_y;
    double _shoulder_const;

    VehicleState _ego_state;
    ros::Time _stamp;

    bool _plan_initialized;

    QuinticPolynomialGeneration _planner;
    PIDController _controller;

    // Planner
    Eigen::MatrixXd _delta_plan;

    delta::utils::FPSLogger _fps_logger = delta::utils::FPSLogger("Planner");

public:
    ros::Publisher control_pub;
    ros::Publisher traj_pub;
    ros::Publisher diag_pub;

    DeltaPlanner(std::string name);

    // void cfgCB(delta_planning_controls::PIDReconfigureConfig &config, uint32_t level);
    void publishControl(VehicleControl control);
    void visualizeEvasiveTrajectory(Eigen::MatrixXd trajectory);
    void publishDiagnostics();

    void run();

    void egoStateCB(const delta_msgs::EgoStateEstimate::ConstPtr& msg);
};

#endif /* _DELTA_PLANNER_H_ */

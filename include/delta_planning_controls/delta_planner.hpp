#ifndef _DELTA_PLANNER_H_
#define _DELTA_PLANNER_H_

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <delta_planning_controls/vehicle_state.hpp>
#include <delta_prediction/EgoStateEstimate.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
// #include <delta_planning_controls/PIDReconfigureConfig.h>
#include <delta_planning_controls/pid_controller.hpp>
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
    VehicleState _ego_state;

    QuinticPolynomialGeneration _planner;
    PIDController _controller;

public:
    DeltaPlanner(std::string name);

    // void cfgCB(delta_planning_controls::PIDReconfigureConfig &config, uint32_t level);

    void egoStateCB(const delta_prediction::EgoStateEstimate::ConstPtr& msg);
};

#endif /* _DELTA_PLANNER_H_ */
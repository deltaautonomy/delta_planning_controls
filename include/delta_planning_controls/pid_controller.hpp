#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <delta_planning_controls/speed_pid.hpp>
#include <delta_planning_controls/stanley_controller.hpp>
#include <delta_planning_controls/vehicle_state.hpp>
#include <eigen3/Eigen/Dense>

class PIDController {
private:
    int _current_path_idx;
    Eigen::MatrixXd _plan;
    int _plan_size;
    SpeedPID longitudinal_controller;
    StanleyController lateral_controller;

    int _findClosestWaypoint(VehicleState ego_state);
    double _findPathSlope(int idx);

public:
    PIDController();

    PIDController(Eigen::MatrixXd plan, double kp, double kd, double ki, double steer_max, double steer_min, double k1, double k2, double dt, double throttle_max, double brake_min);

    VehicleControl runStep(VehicleState ego_state);
};

#endif /* _PID_CONTROLLER_H_ */

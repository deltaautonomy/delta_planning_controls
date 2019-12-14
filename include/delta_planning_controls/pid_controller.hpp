#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <eigen3/Eigen/Dense>
#include <delta_planning_controls/speed_pid.hpp>
#include <delta_planning_controls/stanley_controller.hpp>
#include <delta_planning_controls/vehicle_state.hpp>

class PIDController {
private:
    int _current_path_idx;
    Eigen::MatrixXd _plan;
    bool _valid_plan;
    int _plan_size;
    SpeedPID longitudinal_controller;
    StanleyController lateral_controller;

    int _findClosestWaypoint(VehicleState ego_state);
    double _findPathSlope(int idx);
    double _validator_counter;
    int _validator_idx;
    std::vector<double> _control_validation;

public:
    PIDController();

    PIDController(double kp_max, double kd_max, double ki_max, double kp_min, double kd_min, double ki_min, double steer_max, double steer_min, 
                        double k1, double k2, double dt, double throttle_max, double brake_min);

    void setPlan(Eigen::MatrixXd plan);
    std::vector<double> get_validation();

    VehicleControl runStep(VehicleState ego_state, int _plan_type);
};

#endif /* _PID_CONTROLLER_H_ */

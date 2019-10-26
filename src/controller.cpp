#include <cmath>
#include <iostream>

#include <delta_planning_controls/pid_controller.hpp>

PIDController::PIDController()
{
}

PIDController::PIDController(vector<vector<double>> plan, double kp, double kd, double ki, double steer_max,
    double steer_min, double k1, double k2, double dt, double throttle_max, double brake_min)
{
    _plan(plan);
    lateral_controller = StanleyController(k1, k2, dt, steer_max, steer_min);
    longitudinal_controller = SpeedPID(kp, ki, kd, dt, throttle_max, brake_min);
}

VehicleControl PIDController::runStep(VehicleState ego_state)
{
    int idx = _findClosestWaypoint(ego_state);
    double distance = ego_state.getDistance(_plan(idx, 0), _plan(idx, 1));
    double slope = _findPathSlope(idx);
    double desired_speed = sqrt(pow(_plan(idx, 2), 2) + pow(_plan(idx, 3), 2));

    double ego_speed = ego_state.getVelocity();
    double ego_orientation = ego_state.yaw;

    // get controls
    std::pair<double, double> speed_control = longitudinal_controller.updateError(desired_speed, ego_speed);
    double steering_control = lateral_controller.updateError(slope - ego_orientation, distance, eg0_state.vx);

    VehicleControl ctrl(steering_control, speed_control.first, speed_control.second);

    return ctrl;
}

int PIDController::_findClosestWaypoint(VehicleState ego_state)
{
    double distance = std::numeric_limits<double>::max(), idx = 0;
    for (int i = _current_path_idx; i < _plan.size(); i++) {
        double curent_dist = ego_state.getDistance(plan(i, 0), plan(i, 1));
        if (current_dist < distance) {
            idx = i;
            distance = current_dist;
        }
    }
    _current_path_idx = idx;
    return idx;
}

double PIDController::_findPathSlope(idx)
{
    double slope;
    if ((idx > 0) && (idx < _plan_size - 1))
        slope = (_plan(idx + 1, 1) - _plan(idx - 1, 1)) / (_plan(idx + 1, 0) - _plan(idx - 1, 0));
    else if (idx > 0)
        slope = (_plan(idx, 1) - _plan(idx - 1, 1)) / (_plan(idx, 0) - _plan(idx - 1, 0));
    else
        slope = (_plan(idx + 1, 1) - _plan(idx, 1)) / (_plan(idx + 1, 0) - _plan(idx, 0));

    return slope;
}

#include <cmath>
#include <iostream>

#include <delta_planning_controls/pid_controller.hpp>

using namespace std;

PIDController::PIDController()
{
    _plan_size = 0;
    _valid_plan = false;
}

PIDController::PIDController(double kp, double kd, double ki, double steer_max,
    double steer_min, double k1, double k2, double dt, double throttle_max, double brake_min)
{
    _plan_size = 0;
    lateral_controller = StanleyController(k1, k2, dt, steer_max, steer_min);
    longitudinal_controller = SpeedPID(kp, ki, kd, dt, throttle_max, brake_min);
    _valid_plan = false;

}

void PIDController::setPlan(Eigen::MatrixXd plan)
{
    _plan = plan;
    _valid_plan = true;
    _plan_size = _plan.rows();
    _current_path_idx = 0;
}

VehicleControl PIDController::runStep(VehicleState ego_state)
{ 
    VehicleControl ctrl;
    if ((_valid_plan == true) && (_plan_size > 3)) {
        int idx = _findClosestWaypoint(ego_state);
        // cout<<"Current path index"<<idx<<endl;

        double distance = ego_state.getDistance(_plan(idx, 0), _plan(idx, 1));
        // cout<<"Distance: "<<distance<<endl;
        double slope = _findPathSlope(idx);
        double desired_speed = sqrt(pow(_plan(idx, 2), 2) + pow(_plan(idx, 3), 2));
        double ego_speed = ego_state.getVelocity();
        double ego_orientation = ego_state.yaw;
        double ego_vx = ego_state.vx;

        // get controls
        // cout<<"Ego Orientation "<<ego_orientation<<"          Slope"<<slope<<endl<<endl;
        // cout<<"SPEED: "<<desired_speed<<" "<<ego_speed<<endl;
        std::pair<double, double> speed_control = longitudinal_controller.updateError(desired_speed, ego_speed);
        double steering_control = lateral_controller.updateError(slope - ego_orientation, distance, ego_vx);
        ctrl.setControls(steering_control, speed_control.first, speed_control.second);
        // cout<<"CONTROL: "<<speed_control.first<<" "<<speed_control.second<<" "<<steering_control<<endl;
    } else {
        ctrl.setControls(0, 0, 0);
    }
    return ctrl;
}

// void PIDController::validate_control(VehicleState ego_state)
// {
//     int 
// }

int PIDController::_findClosestWaypoint(VehicleState ego_state)
{
    double distance = std::numeric_limits<double>::max(), current_dist = 0;
    int idx = 0;
    for (int i = _current_path_idx; i < _plan_size; i++) {
        current_dist = ego_state.getDistance(_plan(i, 0), _plan(i, 1));
        if (current_dist < distance) {
            idx = i;
            distance = current_dist;
        }
    }
    _current_path_idx = idx;
    return idx;
}

double PIDController::_findPathSlope(int idx)
{
    double slope;
    if ((idx > 0) && (idx < _plan_size - 1))
        // If the idx corresponds to an internal point on the plan
        slope = (_plan(idx + 1, 1) - _plan(idx - 1, 1)) / (_plan(idx + 1, 0) - _plan(idx - 1, 0));
    else if (idx > 0)
        // If the idx corresponds to the last point on the plan
        slope = (_plan(idx, 1) - _plan(idx - 1, 1)) / (_plan(idx, 0) - _plan(idx - 1, 0));
    else
        // If the idx corresponds to the first point on the plan
        slope = (_plan(idx + 1, 1) - _plan(idx, 1)) / (_plan(idx + 1, 0) - _plan(idx, 0));

    return slope;
}

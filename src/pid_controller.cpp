#include <cmath>
#include <iostream>

#include <delta_planning_controls/pid_controller.hpp>

using namespace std;

PIDController::PIDController()
{
    _plan_size = 0;
    _valid_plan = false;
}

PIDController::PIDController(double kp_max, double kd_max, double ki_max, double kp_min, double kd_min, double ki_min, double steer_max,
    double steer_min, double k1, double k2, double dt, double throttle_max, double brake_min)
{
    _plan_size = 0;
    lateral_controller = StanleyController(k1, k2, dt, steer_max, steer_min);
    longitudinal_controller = SpeedPID(kp_max, ki_max, kd_max, kp_min, ki_min, kd_min, dt, throttle_max, brake_min);
    _valid_plan = false;
    _validator_counter = 0;
    _validator_idx = -1;

}

void PIDController::setPlan(Eigen::MatrixXd plan)
{
    _plan = plan;
    _valid_plan = true;
    _plan_size = _plan.rows();
    _current_path_idx = 0;
    // cout<<"HERE is the plan"<<_plan<<endl;
    // cout<<'\n\n\n\n\n\n\n';
}

VehicleControl PIDController::runStep(VehicleState ego_state, int _plan_type)
{ 
    VehicleControl ctrl;
    if ((_valid_plan == true) && (_plan_size > 3)) {
        int idx = _findClosestWaypoint(ego_state);
        // cout<<"Current path index"<<idx<<endl;

        double distance = ego_state.getDistance(_plan(idx, 0), _plan(idx, 1));
        if (_validator_counter <= _plan_size)
        {
            if (idx != _validator_idx)
            {
                _control_validation.push_back(distance);
                _validator_idx = idx;
            }
            if (idx==_validator_idx)
            {
                if (_control_validation[_control_validation.size()-1] > distance)
                    _control_validation[_control_validation.size()-1] = distance;
            }   
            _validator_counter++;
        }
        // cout<<"Distance: "<<distance<<endl;
        double slope = _findPathSlope(idx);
        double desired_speed = sqrt(pow(_plan(idx, 2), 2) + pow(_plan(idx, 3), 2));
        double ego_speed = ego_state.getVelocity();
        double ego_orientation = ego_state.yaw;
        if (ego_state.yaw < 0)
            ego_orientation += 2*M_PI; 
        double ego_vx = ego_state.vx;

        // get controls
        float orientation_error(0);
        float orientation_error_dir = slope - ego_orientation;
        if (abs(orientation_error_dir) > M_PI)
            orientation_error_dir = -orientation_error_dir;
        float orientation_error_mag = M_PI - abs(abs(slope - ego_orientation) - M_PI);
        if (orientation_error_dir < 0)
            orientation_error = -orientation_error_mag;
        else 
            orientation_error = orientation_error_mag;


        // cout<<"Ego Orientation "<<ego_orientation<<"          Slope"<<slope<<"        "<<orientation_error<<endl;
        // cout<<"Distance Error: "<<distance<<endl;
        std::pair<double, double> speed_control = longitudinal_controller.updateError(desired_speed, ego_speed, _plan_type);
        double steering_control = lateral_controller.updateError(orientation_error, distance, ego_vx);
        ctrl.setControls(steering_control, speed_control.first, speed_control.second);
        // cout<<"CONTROL: "<<speed_control.first<<" "<<speed_control.second<<" "<<steering_control<<endl;


    } else {
        ctrl.setControls(0, 0, 0);
    }
    return ctrl;
}

vector<double> PIDController::get_validation()
{
    float average = accumulate(_control_validation.begin(),_control_validation.end(), 0.0)/_control_validation.size(); 
    cout<<"\033[1;31m*************** Error in Pose: "<<average<<" ***************\033[0m"<<endl;
    // for (int i=0;i<_control_validation.size();i++)
    //     cout<<_control_validation[i]<<" ";
    return _control_validation;
}

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
        slope = atan2(_plan(idx + 1, 1) - _plan(idx - 1, 1), _plan(idx + 1, 0) - _plan(idx - 1, 0));
    else if (idx > 0)
        // If the idx corresponds to the last point on the plan
        slope = atan2(_plan(idx, 1) - _plan(idx - 1, 1) , _plan(idx, 0) - _plan(idx - 1, 0));
    else
        // If the idx corresponds to the first point on the plan
        slope = atan2(_plan(idx + 1, 1) - _plan(idx, 1) , _plan(idx + 1, 0) - _plan(idx, 0));

    return slope > 0 ? slope : (2*M_PI + slope);
}

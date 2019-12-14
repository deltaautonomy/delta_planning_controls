#include <cmath>
#include <iostream>

#include <delta_planning_controls/speed_pid.hpp>

using namespace std;

SpeedPID::SpeedPID()
{
    _Kp_max = 0;
    _Kd_max = 0;
    _Ki_max = 0;

    _Kp_min = 0;
    _Kd_min = 0;
    _Ki_min = 0;

    _max = 1;
    _min = -1;
    _dt = 0.1;
    _last_error = 0;
    _integral_error = 0;
}

SpeedPID::SpeedPID(double kp_max, double kd_max, double ki_max, double kp_min, double kd_min, double ki_min,
                 double dt, double max, double min) : _Kp_max(kp_max), _Kd_max(kd_max), _Ki_max(ki_max), _Kp_min(kp_min), _Kd_min(kd_min), _Ki_min(ki_min), _dt(dt), _max(max), _min(min)
{
    // @TODO: Handle max min random values
    _last_error = 0;
    _integral_error = 0;
}

pair<double, double> SpeedPID::_getThrottleBrake(double control)
{
    double throttle = 0.0, brake = 0.0;
    if (control > 0.0) {
        if (control < _max)
            throttle = control;
        else
            throttle = _max;
    } else {
        if (control > _min)
            brake = control;
        else
            brake = _min;
    }

    return make_pair(throttle, brake);
}

pair<double, double> SpeedPID::updateError(double desired_speed, double current_speed, int _plan_type)
{
    double error = desired_speed - current_speed;
    _integral_error += error * _dt;

    double output;
    if(_plan_type == 2)
    {
        output = _Kp_max * error + _Ki_max * _integral_error + _Kd_max * (error - _last_error) / _dt;
    }
    else
    {
        output = _Kp_min * error + _Ki_min * _integral_error + _Kd_min * (error - _last_error) / _dt;
    }
    // cout<<"Error in Speed: "<<error<<" Output: "<<output<<endl;

    _last_error = error;

    return _getThrottleBrake(output);
}
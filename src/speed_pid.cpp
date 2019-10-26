#include <cmath>
#include <iostream>

#include <delta_planning_controls/speed_pid.hpp>

using namespace std;

SpeedPID::SpeedPID()
{
    _Kp = 0;
    _Kd = 0;
    _Ki = 0;
    _max = 1;
    _min = -1;
    _dt = 0.1;
    _last_error = 0;
    _integral_error = 0;
}

SpeedPID::SpeedPID(double kp, double kd, double ki, double dt, double max, double min)
    : _Kp(kp)
    , _Kd(kd)
    , _Ki(ki)
    , _dt(dt)
    , _max(max)
    , _min(min)
{
    // @TODO: Handle max min random values
    _last_error = 0;
    _integral_error = 0;
}

pair<double, double> SpeedPID::_getThrottleBrake(double control)
{
    double throttle = 0, brake = 0;
    if (control > 0) {
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

pair<double, double> SpeedPID::updateError(double desired_speed, double current_speed)
{
    double error = desired_speed - current_speed;
    _integral_error += error * _dt;

    double output = _Kp * error + _Ki * _integral_error + _Kd * (error - _last_error) / _dt;

    _last_error = error;

    return _getThrottleBrake(output);
}
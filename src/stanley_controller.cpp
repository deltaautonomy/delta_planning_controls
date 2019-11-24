#include <cmath>
#include <iostream>

#include <delta_planning_controls/stanley_controller.hpp>

using namespace std;

StanleyController::StanleyController()
{
    _K1 = 0;
    _K2 = 0;
    _max = 1;
    _min = -1;
    _dt = 0.1;
}

StanleyController::StanleyController(double k1, double k2, double dt, double max, double min)
    : _K1(k1)
    , _K2(k2)
    , _dt(dt)
    , _max(max)
    , _min(min)
{
}

double StanleyController::updateError(double orientation_error, double cross_track_error, double longitudinal_velocity)
{
    double output = 0.0;
    if(longitudinal_velocity > 0.001)
        output =  - _K1 * orientation_error + _K2 * atan2( cross_track_error,longitudinal_velocity);
    else
        return output;

    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    return output;
}
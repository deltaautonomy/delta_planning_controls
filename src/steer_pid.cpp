#include <cmath>
#include <iostream>

#include <delta_planning_controls/StanleyController.hpp>

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
{
    _K1(k1);
    _K2(k2);
    _dt(dt);
    _max(max;
    _min(min);
}

double StanleyController::updateError(double orientation_error, double cross_track_error, double longitudinal_velocity)
{
    double output = _K1 * orientation_error + tan(_K2 * cross_track_error / current_speed);

    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    return output;
}
#ifndef _SPEED_PID_H_
#define _SPEED_PID_H_

#include <utility>

class SpeedPID {

private:
    // Constants
    double _dt;
    double _max;
    double _min;

    // Tunable Constants
    double _Kp_min;
    double _Kd_min;
    double _Ki_min;
    double _Kp_max;
    double _Kd_max;
    double _Ki_max;

    // Error
    double _last_error;
    double _integral_error;

    std::pair<double, double> _getThrottleBrake(double control);

public:
    SpeedPID();

    SpeedPID(double kp_max, double kd_max, double ki_max, double kp_min, double kd_min, double ki_min, double dt, double max, double min);

    std::pair<double, double> updateError(double desired_speed, double current_speed, int _plan_type);
};

#endif /* _SPEED_PID_H_ */

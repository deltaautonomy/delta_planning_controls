#include <utility>

#ifndef _SPEED_PID_H_
#define _SPEED_PID_H_

class SpeedPID {

private:
    // Constants
    double _dt;
    double _max;
    double _min;

    // Tunable Constants
    double _Kp;
    double _Kd;
    double _Ki;

    // Error
    double _last_error;
    double _integral_error;

    std::pair<double, double> _getThrottleBrake(double control);

public:
    SpeedPID();

    SpeedPID(double kp, double kd, double ki, double dt, double max, double min);

    std::pair<double, double> updateError(double desired_speed, double current_speed);
};

#endif /* _SPEED_PID_H_ */

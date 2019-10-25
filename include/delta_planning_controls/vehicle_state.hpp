#ifndef _VEHICLE_STATE_H_
#define _VEHICLE_STATE_H_

#include <cmath>

struct VehicleState {
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double yaw_rate;

    double get_distance(double pos_x, double pos_y)
    {
        return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
    }

    double get_velocity()
    {
        return sqrt(pow(vx, 2) + pow(vy, 2));
    }
};

struct VehicleControl {
    double steer;
    double throttle;
    double brake;
};

#endif /* _VEHICLE_STATE_H_ */

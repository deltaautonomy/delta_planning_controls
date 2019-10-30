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
    double acc_x;
    double acc_y;

    VehicleState() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), yaw_rate(0.0), acc_x(0.0), acc_y(0.0)
    {
    }

    VehicleState(double inp_x, double inp_y, double inp_yaw, double inp_vx, double inp_vy, double inp_yaw_rate, 
        double inp_acc_x, double inp_acc_y) : x(inp_x), y(inp_y), yaw(inp_yaw), vx(inp_vx), vy(inp_vy), 
                                                yaw_rate(inp_yaw_rate), acc_x(inp_acc_y), acc_y(inp_acc_y)
    {
    }

    double getDistance(double pos_x, double pos_y)
    {
        return sqrt(pow((pos_x - x), 2) + pow((pos_y - y), 2));
    }

    double getVelocity()
    {
        return sqrt(pow(vx, 2) + pow(vy, 2));
    }
};

struct VehicleControl {
    double steer;
    double throttle;
    double brake;

    VehicleControl() : steer(0.0), throttle(0.0), brake(0.0) 
    {
    }

    VehicleControl(double inp_steer, double inp_throttle, double inp_brake) : steer(inp_steer), throttle(inp_throttle), brake(inp_brake) 
    {
    }
};

#endif /* _VEHICLE_STATE_H_ */

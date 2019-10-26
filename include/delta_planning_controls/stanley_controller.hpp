#ifndef _STANLEY_CONTROLLER_H_
#define _STANLEY_CONTROLLER_H_

class StanleyController {

private:
    // Constants
    double _dt;
    double _max;
    double _min;

    // Tunable Constants
    double _K1;
    double _K2;

public:
    StanleyController();

    StanleyController(double k1, double k2, double dt, double max, double min);

    double updateError(double orientation_error, double cross_track_error, double longitudinal_velocity);
};

#endif /* _STANLEY_CONTROLLER_H_ */

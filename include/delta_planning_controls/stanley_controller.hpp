#ifndef _STANLEY_CONTROLLER_H_
#define _STANLEY_CONTROLLER_H_

#include <cmath>
#include <iostream>

class StanleyController {

private:
    // Constants
    double _dt;
    double _max;
    double _min;

    // Tunable Constants
    double _K1;
    double _K2;

    // Error
    double _error;

public:
    StanleyController();

    StanleyController(double k1, double k2);

    void update_error();
};

#endif /* _STEER_PID_H_ */

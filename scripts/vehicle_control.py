import numpy as np
import matplotlib.pyplot as plt

class VehicleControl:
    '''Planning and Controls Subsystem'''
    def __init__(self, ctrl_freq, ttc):
        self.ctrl_freq = ctrl_freq
        self.ttc = ttc
        dt = 1/self.ctrl_freq # Define time step

    # Method to generate reference trajectory
    def generate_evasive_traj(self,boundary_vals):
        ''' Generates a minimum jerk trajectory in x and y'''
        xi,yi,xf,yf,vxi,vyi,axi,ayi = boundary_vals
        x_traj = [] # List of list of position and velocity at ctrl_freq in x
        y_traj = [] # List of list of position and velocity at ctrl_freq in y

        # minimum jerk trajectory is a 5th order polynomial
        # y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # Given initial and final values in pos, vel and acc (Note final acc is 0 and final vel is 0) coeffs are:
        T = self.ttc
        a0x = xi
        a1x = vxi
        a2x = axi/2
        a3x = -(20*xi - 20*xf + 12*T*vxi + 11*axi*T**2 - 8*axi*T)/(2*T**3)
        a4x = (30*xi - 30*xf + 16*T*vxi + 17*axi*T**2 - 14*axi*T)/(2*T**4)
        a5x = -(12*xi - 12*xf + 6*T*vxi + 7*axi*T**2 - 6*axi*T)/(2*T**5)
        # Similarly for y
        a0y = yi
        a1y = vyi
        a2y = ayi/2
        a3y = -(20*yi - 20*yf + 12*T*vyi + 11*ayi*T**2 - 8*ayi*T)/(2*T**3)
        a4y = (30*yi - 30*yf + 16*T*vyi + 17*ayi*T**2 - 14*ayi*T)/(2*T**4)
        a5y = -(12*yi - 12*yf + 6*T*vyi + 7*ayi*T**2 - 6*ayi*T)/(2*T**5)

        dt = 1/self.ctrl_freq # Define time step
        t = dt # Current time
        # Populate trajectory information
        while t <= self.ttc:
            # Trajectory in x
            x_t = a0x + a1x*t + a2x*t**2 + a3x*t**3 + a4x*t**4 + a5x*t**5
            vx_t = a1x + 2*a2x*t + 3*a3x*t**2 + 4*a4x*t**3 + 5*a5x*t**4
            x_traj.append([x_t, vx_t])
            # Trajectory in y
            y_t = a0y + a1y*t + a2y*t**2 + a3y*t**3 + a4y*t**4 + a5y*t**5
            vy_t = a1y + 2*a2y*t + 3*a3y*t**2 + 4*a4y*t**3 + 5*a5y*t**4
            y_traj.append([y_t, vy_t])

            t = t + dt# Increment time

        return np.asarray(x_traj), np.asarray(y_traj)

if __name__ == "__main__":
    delta_control = VehicleControl(10,2)
    boundary_vals = [0,0,10,10,0,0,0,0]
    x_traj,y_traj = delta_control.generate_evasive_traj(boundary_vals)
    print(x_traj)
    print('\n')
    print(y_traj)

    # plt.scatter(x_traj[:,],y_traj[:,])
    # plt.show()
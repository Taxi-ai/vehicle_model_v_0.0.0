// global Kinematic model
#include <math.h>
#include <iostream>
#include "Dense"

//helper functions
double pi() {return M_PI;}
double deg2rad(double x) {return x * pi() / 180 ;}
double rad2deg(double x) {return x * 180 / pi() ;}

/*  LF :measures the distance 
between the center of mass of the vehicle and it's front axle.
The larger the vehicle, the slower the turn rate. */
const double Lf = 2;

// Global kinematic vector functions to implement model equations
// State is [x, y, psi, v] ,, psi :vehicle orientation
// Actuators is [delta, a] ,, delta : steering angle

Eigen::VectorXd globalKinematic(Eigen::VectorXd state,Eigen::VectorXd actuators, double dt)
{
    //next_state vector size reservation
    Eigen::VectorXd next_state(state.size());
    //next state eqn_s
    next_state[0] = state[0] + state[3] * cos(state[2])*dt;  //x(t+1) = x(t) + v(t)*cos(psi)*dt
    next_state[1] = state[1] + state[3] * sin(state[2])*dt;  //y(t+1) = y(t) + v(t)*sin(psi)*dt
    next_state[2] = state[2] + state[3]/Lf *actuators[0]*dt; //psi(t+1) = psi(t) + v(t) / Lf *delta*dt
    next_state[3] = state[3] + actuators[1]*dt;              //v(t+1) = v(t) + a(t)*dt

    return next_state;
}
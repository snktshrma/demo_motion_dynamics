#ifndef _ISS_DYNAMICS_H
#define _ISS_DYNAMICS_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

class iss_dynamics {

public:
    iss_dynamics() {};
    struct state
    {
        double x, y, z;
        double vx, vy, vz;
    };

    struct dotstate
    {
        double dx, dy, dz;
        double ddx, ddy, ddz;
    };

    struct ssParm
    {
        double mu;
        double rOrbit;
        double temp;
        double rho;
        double Cd;
        double A;
        double pressure;
        double mass;
    };

    // External Torques and Forces
    Eigen::Vector3d atmDrag(const state& x, const ssParm& par);
    Eigen::Vector3d bodyAccel(const state& x, const ssParm& par);


    Eigen::Matrix<double,3,3> eulerKinMat(double phi, double theta);

    Eigen::VectorXd compute(const state& x, const ssParm& par);

    state rk4Step(const state& x, const ssParm& par, double dt);

};

#endif //_ISS_DYNAMICS_H
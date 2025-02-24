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
        double p, q, r;
        double phi, theta, psi;
        Eigen::VectorXd deltas;
        float alt;
    };

    struct dotstate
    {
        double dp, dq, dr;
        double dphi, dtheta, dpsi;
        Eigen::VectorXd ddeltas;
        float dalt;
    };

    struct ssParm
    {
        double Ixx, Iyy, Izz;
        double Ixy, Ixz, Iyz;
        double mu;
        double rOrbit;
        Eigen::Vector3d T_cmg;
    };

    Eigen::Matrix<double,3,3> iMat(const ssParm& par);
    Eigen::Vector3d compute_h(const Eigen::VectorXd& delta);
    Eigen::Matrix<double,4,3> jacobian(Eigen::VectorXd delta);

    // External Torques and Forces
    Eigen::Vector3d gravityGradT(const state& x, const ssParm& par);
    // Eigen::Vector3d gravityGradT(const state& x, const ssParm& par);
    // Eigen::Vector3d gravityGradT(const state& x, const ssParm& par);


    Eigen::Matrix<double,3,3> eulerKinMat(double phi, double theta);

    Eigen::VectorXd compute(const state& x, const ssParm& par);

    state rk4Step(const state& x, const ssParm& par, double dt);

};

#endif //_ISS_DYNAMICS_H
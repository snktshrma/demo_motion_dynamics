#include "demo_motion_dynamics/iss_dynamics_orbit.hpp"
#include "demo_motion_dynamics/L_p_func.h"

Eigen::Vector3d iss_dynamics::atmDrag(const state& x, const ssParm& par)
{
    double mol_mass = 2.66e-26; // oxygen dominated
    double kb =  1.380649e-23;

    // I've modelled density using ideal gas law. It can also be approximated using an exponential 
    // model wrt to the thermosphere environmental condition: rho0 * e^(-(h-h0)/H)

    // Assumption: Solar activity and other factors kept constant
    double therm_dens = 4e-10; //kg/m^3
    double h0 = 400; //km
    double H = 70; //km; ever incrase in altitude by H, the density thins out oy a factor of e

    Eigen::Vector3d v(x.vx, x.vy, x.vz);
    double mag = v.norm();

    double rho = par.pressure * mol_mass/ (par.temp * kb);
    // std::cout << rho << ", " << (1/par.mass) << ", " << pow(mag,2) << ", " << rho << ", " << par.Cd << ", " << par.A << ", " << (v/mag) << std::endl;


    return -0.5 * (1/par.mass) * pow(mag,2) * rho * par.Cd * par.A * (v/mag) * 10;
}


Eigen::Vector3d iss_dynamics::bodyAccel(const state& x, const ssParm& par)
{
    double rVec = -par.mu/std::pow((std::pow(x.x,2) + std::pow(x.y,2) + std::pow(x.z,2)),1.5);

    return rVec * Eigen::Vector3d(x.x, x.y, x.z);
}

Eigen::VectorXd iss_dynamics::compute(const state& x, const ssParm& par)
{
    // Eigen::Vector3d w(x.x, x.y, x.z);
    
    Eigen::Vector3d A_ext = atmDrag(x, par);
    Eigen::Vector3d w_dot = bodyAccel(x, par);
    Eigen::Vector3d vel_dot = w_dot + A_ext;
 
    Eigen::VectorXd xDot(6);
    xDot(0) = x.vx;
    xDot(1) = x.vy;
    xDot(2) = x.vz;
    xDot(3) = vel_dot(0);
    xDot(4) = vel_dot(1);
    xDot(5) = vel_dot(2);

    return xDot;
}

iss_dynamics::state iss_dynamics::rk4Step(const state& x, const ssParm& par, double dt)
{
    auto f = [&](const state& s){ return compute(s, par); };

    auto stateToVec = [&](const state& s){
        Eigen::VectorXd v(6);
        v << s.x, s.y, s.z, s.vx, s.vy, s.vz;
        return v;
    };

    auto vecToState = [&](const Eigen::VectorXd& v, const state& ref){
        state s;
        s.x     = v(0);
        s.y     = v(1);
        s.z     = v(2);
        s.vx    = v(3);
        s.vy    = v(4);
        s.vz    = v(5);
        return s;
    };

    Eigen::VectorXd xVec = stateToVec(x);

    Eigen::VectorXd k1 = f(x);
    Eigen::VectorXd x2 = xVec + 0.5 * dt * k1;
    state s2 = vecToState(x2, x);
    Eigen::VectorXd k2 = f(s2);

    Eigen::VectorXd x3 = xVec + 0.5 * dt * k2;
    state s3 = vecToState(x3, x);
    Eigen::VectorXd k3 = f(s3);

    Eigen::VectorXd x4 = xVec + dt * k3;
    state s4 = vecToState(x4, x);
    Eigen::VectorXd k4 = f(s4);

    Eigen::VectorXd xNext = xVec + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    return vecToState(xNext, x);
}

#include "demo_motion_dynamics/iss_dynamics_orbit.hpp"
#include "demo_motion_dynamics/L_p_func.h"

/**
 * @brief Calculates the acceleration due to atmospheric drag.
 * 
 * This function computes the atmospheric drag force on the spacecraft using the 
 * ideal gas law to determine the density and applying the drag equation.
 * 
 * @param x The current state containing position and velocity.
 * @param par The spacecraft and environmental parameters.
 * @return Eigen::Vector3d The acceleration due to atmospheric drag.
 */
Eigen::Vector3d iss_dynamics::atmDrag(const state& x, const ssParm& par)
{
    // Constants for the atmosphere (assumes oxygen dominated atmosphere)
    double mol_mass = 2.66e-26; // Molecular mass in kg (approximate for oxygen)
    double kb =  1.380649e-23;  // Boltzmann constant in J/K

    double therm_dens = 4e-10;  // Density in kg/m^3 (example constant value)
    double h0 = 400;            // Reference altitude in km
    double H = 70;              // Scale height in km (density falls off by a factor of e per H)

    Eigen::Vector3d v(x.vx, x.vy, x.vz);
    double mag = v.norm();

    double rho = par.pressure * mol_mass / (par.temp * kb);

    // std::cout << rho << ", " << (1/par.mass) << ", " << pow(mag,2) << ", " << rho << ", " << par.Cd << ", " << par.A << ", " << (v/mag) << std::endl;

    return -0.5 * (1 / par.mass) * pow(mag, 2) * rho * par.Cd * par.A * (v / mag) * 10;
}

/**
 * @brief Computes the gravitational acceleration acting on the spacecraft.
 * 
 * This function calculates the acceleration due to Earth's gravity using Newton's law 
 * of universal gravitation.
 * 
 * @param x The current state containing the position of the spacecraft.
 * @param par The spacecraft parameters including Earth's gravitational parameter.
 * @return Eigen::Vector3d The gravitational acceleration vector.
 */
Eigen::Vector3d iss_dynamics::bodyAccel(const state& x, const ssParm& par)
{
    double r = std::sqrt(std::pow(x.x, 2) + std::pow(x.y, 2) + std::pow(x.z, 2));
    
    double rVec = -par.mu / std::pow(r, 3);
    
    return rVec * Eigen::Vector3d(x.x, x.y, x.z);
}

/**
 * @brief Computes the derivative of the state vector.
 * 
 * This function combines both gravitational and atmospheric drag accelerations
 * to compute the derivative of the state vector for the simulation.
 * 
 * @param x The current state vector.
 * @param par The spacecraft parameters.
 * @return Eigen::VectorXd The derivative of the state vector (6 elements).
 */
Eigen::VectorXd iss_dynamics::compute(const state& x, const ssParm& par)
{
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

/**
 * @brief Advances the state vector one time step using the Runge-Kutta 4th order method.
 * 
 * This function implements the RK4 integration algorithm to numerically integrate 
 * the state vector forward in time.
 * 
 * @param x The current state of the spacecraft.
 * @param par The spacecraft parameters.
 * @param dt The time step for integration.
 * @return iss_dynamics::state The state of the spacecraft after one time step.
 */
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
        s.x  = v(0);
        s.y  = v(1);
        s.z  = v(2);
        s.vx = v(3);
        s.vy = v(4);
        s.vz = v(5);
        return s;
    };

    Eigen::VectorXd xVec = stateToVec(x);

    // RK4 integration steps
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

    Eigen::VectorXd xNext = xVec + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    return vecToState(xNext, x);
}

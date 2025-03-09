#ifndef _ISS_DYNAMICS_H
#define _ISS_DYNAMICS_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

/**
 * @brief A class to model the dynamics of the ISS or similar spacecraft.
 *
 * This class contains state definitions, parameters, and functions to compute 
 * forces, accelerations, and perform numerical integration (using RK4) for 
 * orbit dynamics, including atmospheric drag and gravitational effects.
 */
class iss_dynamics {

public:
    /**
     * @brief Default constructor.
     */
    iss_dynamics() {};

    /**
     * @brief Structure representing the state of the spacecraft.
     */
    struct state
    {
        double x, y, z;   ///< Position coordinates (m)
        double vx, vy, vz;///< Velocity components (m/s)
    };

    /**
     * @brief Structure representing the derivative of the state.
     */
    struct dotstate
    {
        double dx, dy, dz;    ///< Time derivatives of position (velocity components)
        double ddx, ddy, ddz; ///< Time derivatives of velocity (acceleration components)
    };

    /**
     * @brief Structure containing spacecraft and environmental parameters.
     */
    struct ssParm
    {
        double mu;       ///< Gravitational parameter (m^3/s^2)
        double rOrbit;   ///< Nominal orbit radius (m)
        double temp;     ///< Atmospheric temperature (K)
        double rho;      ///< Atmospheric density (kg/m^3)
        double Cd;       ///< Drag coefficient
        double A;        ///< Cross-sectional area (m^2)
        double pressure; ///< Atmospheric pressure (Pa)
        double mass;     ///< Spacecraft mass (kg)
    };

    /**
     * @brief Computes the atmospheric drag acceleration on the spacecraft.
     * 
     * This function calculates the acceleration due to atmospheric drag using 
     * the current state and parameters.
     *
     * @param x Current state of the spacecraft.
     * @param par Spacecraft and environmental parameters.
     * @return Eigen::Vector3d Atmospheric drag acceleration vector.
     */
    Eigen::Vector3d atmDrag(const state& x, const ssParm& par);

    /**
     * @brief Computes the gravitational acceleration acting on the spacecraft.
     * 
     * This function calculates the acceleration due to Earth's gravity.
     *
     * @param x Current state of the spacecraft.
     * @param par Spacecraft parameters (including Earth's gravitational parameter).
     * @return Eigen::Vector3d Gravitational acceleration vector.
     */
    Eigen::Vector3d bodyAccel(const state& x, const ssParm& par);

    /**
     * @brief Computes the derivative of the state vector.
     * 
     * This function combines gravitational and atmospheric drag accelerations 
     * to calculate the derivative of the state.
     *
     * @param x Current state of the spacecraft.
     * @param par Spacecraft and environmental parameters.
     * @return Eigen::VectorXd A 6-element vector representing state derivative.
     */
    Eigen::VectorXd compute(const state& x, const ssParm& par);

    /**
     * @brief Advances the state vector by one time step using the Runge-Kutta 4th order method.
     * 
     * This function uses RK4 integration to propagate the state forward in time.
     *
     * @param x Current state of the spacecraft.
     * @param par Spacecraft and environmental parameters.
     * @param dt Time step for integration (s).
     * @return state The updated state after one time step.
     */
    state rk4Step(const state& x, const ssParm& par, double dt);

};

#endif //_ISS_DYNAMICS_H

#include "demo_motion_dynamics/iss_dynamics_orbit.hpp"
#include <fstream>
#include <iostream>
// #include <rclcpp/rclcpp.hpp>

/**
 * @brief Main function to simulate the spacecraft's orbital dynamics.
 *
 * This function initializes the spacecraft's state for a circular orbit,
 * sets up simulation parameters, and performs numerical integration using 
 * the Runge-Kutta 4th order method. The results are saved to a CSV file.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line argument vector.
 * @return int Exit status code.
 */
int main(int argc, char ** argv)
{
    // Initialize ROS if needed
    // rclcpp::init(argc, argv);

    iss_dynamics dyn;

    iss_dynamics::state x0;
    x0.x  = 0.0;      // Initial x position at the orbit radius (m)
    x0.y  = 0.0;
    x0.z  = 7.0e6;    // Initial z position (m)
    
    double mu     = 3.986004418e14; // Earth's gravitational parameter (m^3/s^2)
    double rOrbit = 7.0e6;           // Orbit radius (m)
    double v_circ = std::sqrt(mu / rOrbit); // Circular orbit speed (m/s)
    double v_elip = std::sqrt(mu * ((2/rOrbit) - (1/7.5e-6)));
    
    x0.vx = v_circ * std::cos(0.3);
    x0.vy = v_circ * std::sin(0.3);
    x0.vz = 0.0;

    iss_dynamics::ssParm par;
    par.mu = mu;
    par.rOrbit = rOrbit;
    par.Cd = 2.2;          // Drag coefficient
    par.A = 4;             // Cross-sectional area (m^2)
    par.mass = 500;        // Mass of the spacecraft (kg)
    par.pressure = 1.57e-5; // Atmospheric pressure (as per NRLMSISE-00, in appropriate units)
    par.temp = 2000;       // Atmospheric temperature (K)

    double dt = 0.1;       // Time step (s)
    int steps = 100000;    // Number of integration steps
    iss_dynamics::state x = x0; // Current state initialized with x0

    std::ofstream file("ss_data.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening output file." << std::endl;
        return -1;
    }

    file << "time,x,y,z,vx,vy,vz\n";

    for (int i = 0; i <= steps; ++i)
    {
        double t = i * dt;
        file << t << "," 
             << x.x << "," << x.y << "," << x.z << ","
             << x.vx << "," << x.vy << "," << x.vz << "\n";

        if (i < steps) {
            double earthRadius = 6378137.0; // Earth's radius in meters
            double r = std::sqrt(x.x * x.x + x.y * x.y + x.z * x.z);
            
            if (r <= earthRadius) {
                x.x *= earthRadius / r;
                x.y *= earthRadius / r;
                x.z *= earthRadius / r;
                x.vx = 0.0;
                x.vy = 0.0;
                x.vz = 0.0;
            }
            x = dyn.rk4Step(x, par, dt);
        }
    }

    file.close();

    return 0;
}

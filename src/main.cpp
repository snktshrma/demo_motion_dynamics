#include "demo_motion_dynamics/iss_dynamics_orbit.hpp"
#include <fstream>
#include <iostream>
// #include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    // rclcpp::init(argc, argv);

    iss_dynamics dyn;

    iss_dynamics::state x0;
    x0.x  = 0.0;      // initial x at the orbit radius
    x0.y  = 0.0;
    x0.z  = 7.0e6;
    // For a circular orbit, velocity magnitude v = sqrt(mu / rOrbit)
    // vel vector perpendicular to pos vector for circular orbit.
    double mu     = 3.986004418e14;
    double rOrbit = 7.0e6;
    double v_circ = std::sqrt(mu / rOrbit);
    double v_elip = std::sqrt(mu*((2/rOrbit) - (1/7.5e-6)));
    x0.vx = v_circ * std::cos(0.3);
    x0.vy = v_circ * std::sin(0.3);
    x0.vz = 0.0;

    iss_dynamics::ssParm par;
    par.mu = mu;
    par.rOrbit = rOrbit;
    par.Cd = 2.2;
    par.A = 4; //m^2
    par.mass = 500; //kg
    par.pressure = 1.57e-5; //as per NRLMSISE-00
    par.temp = 2000; //K

    double dt = 0.1;
    int steps = 100000;
    iss_dynamics::state x = x0;

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
            double earthRadius = 6378137.0;
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

#include "demo_motion_dynamics/iss_dynamics_orbit.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  iss_dynamics dyn;

  iss_dynamics::state x0;
  x0.p = 0.0;
  x0.q = 0.001;
  x0.r = 0.0;
  x0.phi = 0.0;
  x0.theta = 0.0;
  x0.psi = 0.0;
  x0.deltas = Eigen::VectorXd::Zero(4);

  iss_dynamics::ssParm par;
  par.Ixx = 10.0;
  par.Iyy = 12.0;
  par.Izz = 9.0;
  par.Ixy = 0.0;
  par.Ixz = 0.0;
  par.Iyz = 0.0;
  par.mu = 3.986004418e14;
  par.rOrbit = 7.0e6;
  par.T_cmg << 1, 1, 0;

  double dt = 0.1;
  int steps = 7000;
  iss_dynamics::state x = x0;

  std::ofstream file("ss_data.csv");
  file << "time,p,q,r,phi,theta,psi,delta1,delta2,delta3,delta4\n";

  for(int i = 0; i <= steps; ++i)
  {
      double t = i * dt;
      file << t << "," 
           << x.p << "," << x.q << "," << x.r << ","
           << x.phi << "," << x.theta << "," << x.psi << ","
           << x.deltas(0) << "," << x.deltas(1) << "," << x.deltas(2) << "," << x.deltas(3) << "\n";

      if(i < steps) x = dyn.rk4Step(x, par, dt);
  }

  file.close();
  return 0;
}

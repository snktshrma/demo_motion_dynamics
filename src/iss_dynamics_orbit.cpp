#include "demo_motion_dynamics/iss_dynamics_orbit.hpp"
#include "demo_motion_dynamics/L_p_func.h"


// iss_dynamics::iss_dynamics() {
// }

Eigen::Matrix<double,3,3> iss_dynamics::iMat(const ssParm& par)
{
    Eigen::Matrix<double,3,3> Ib;
    Ib << par.Ixx, par.Ixy, par.Ixz,
          par.Ixy, par.Iyy, par.Iyz,
          par.Ixz, par.Iyz, par.Izz;
    
    return Ib;
}

Eigen::Vector3d iss_dynamics::compute_h(const Eigen::VectorXd& delta) {
    constexpr double beta = 54.73 * M_PI / 180.0;

    double delta_1 = delta(0);
    double delta_2 = delta(1);
    double delta_3 = delta(2);
    double delta_4 = delta(3);

    Eigen::Vector3d r1, r2, r3, r4;

    r1 << -std::cos(beta) * std::sin(delta_1),
           std::cos(delta_1),
           std::sin(beta) * std::sin(delta_1);

    r2 << -std::cos(delta_2),
          -std::cos(beta) * std::sin(delta_2),
           std::sin(beta) * std::sin(delta_2);

    r3 <<  std::cos(beta) * std::sin(delta_3),
          -std::cos(delta_3),
           std::sin(beta) * std::sin(delta_3);

    r4 <<  std::cos(delta_4),
           std::cos(beta) * std::sin(delta_4),
           std::sin(beta) * std::sin(delta_4);

    Eigen::Vector3d h = 10 * (r1 + r2 + r3 + r4);

    return h;
}


Eigen::Matrix<double,4,3> iss_dynamics::jacobian(Eigen::VectorXd delta) {
    Eigen::Matrix<double,4,3> jacob(4,3);
    double out[12];
    L_p_func(delta(0),delta(1),delta(2),delta(3),out);

    jacob = Eigen::Map<Eigen::Matrix<double,4,3>>(out);
    return jacob;
}

Eigen::Vector3d iss_dynamics::gravityGradT(const state& x, const ssParm& par)
{
    double n = std::sqrt(par.mu / std::pow(par.rOrbit, 3));
    double sp = std::sin(x.phi),   cp = std::cos(x.phi);
    double st = std::sin(x.theta), ct = std::cos(x.theta);

    double T1 = (par.Iyy - par.Izz) * st * ct;
    double T2 = (par.Izz - par.Ixx) * sp * cp;
    double T3 = (par.Ixx - par.Iyy) * sp * st;

    return 3.0 * n * n * Eigen::Vector3d(T1, T2, T3);
}

Eigen::Matrix<double,3,3> iss_dynamics::eulerKinMat(double phi, double theta)
{
    Eigen::Matrix<double,3,3> M;
    double sp = std::sin(phi), cp = std::cos(phi);
    double st = std::sin(theta), ct = std::cos(theta);
    double tt = st / ct;

    M << 1.0,     sp * tt,    cp * tt,
         0.0,     cp,         -sp,
         0.0, sp / ct,   cp / ct;

    return M;
}

Eigen::VectorXd iss_dynamics::compute(const state& x, const ssParm& par)
{
    Eigen::Vector3d w(x.p, x.q, x.r);
    Eigen::Matrix<double,3,3> Ib = iMat(par);
    Eigen::Matrix<double,3,3> IbInv = Ib.inverse();

    Eigen::Vector3d T_ext = gravityGradT(x, par);
    Eigen::Vector3d T_cmg = par.T_cmg;
    Eigen::Vector3d crossTerm = w.cross(Ib * w);
    Eigen::Vector3d w_dot = IbInv * (T_ext + T_cmg - crossTerm);

    Eigen::Matrix<double,3,3> M = eulerKinMat(x.phi, x.theta);
    Eigen::Vector3d euler_dot = M * w;

    int nCmg = (int)x.deltas.size();

    // TODO: Add Jacobian overall calculation
    Eigen::Matrix<double,4,3> jacob = jacobian(x.deltas);
    Eigen::Vector3d h = compute_h(x.deltas);
    // std::cout << "----------------" << std::endl;
    // std::cout << jacob << std::endl;
    // std::cout << -(T_cmg+w.cross(h)) << std::endl;
    Eigen::VectorXd delta_dots = jacob*(-(T_cmg+w.cross(h)));
    // Eigen::VectorXd delta_dots = Eigen::VectorXd::Zero(nCmg);

    Eigen::VectorXd xDot(6 + nCmg);
    xDot(0) = w_dot(0);
    xDot(1) = w_dot(1);
    xDot(2) = w_dot(2);
    xDot(3) = euler_dot(0);
    xDot(4) = euler_dot(1);
    xDot(5) = euler_dot(2);
    for(int i = 0; i < nCmg; ++i) xDot(6 + i) = delta_dots(i);

    return xDot;
}

iss_dynamics::state iss_dynamics::rk4Step(const state& x, const ssParm& par, double dt)
{
    auto f = [&](const state& s){ return compute(s, par); };

    auto stateToVec = [&](const state& s){
        int nCmg = (int)s.deltas.size();
        // std::cout << s.deltas.size();
        Eigen::VectorXd v(6 + nCmg);
        v << s.p, s.q, s.r, s.phi, s.theta, s.psi, s.deltas;
        return v;
    };

    auto vecToState = [&](const Eigen::VectorXd& v, const state& ref){
        state s;
        s.p     = v(0);
        s.q     = v(1);
        s.r     = v(2);
        s.phi   = v(3);
        s.theta = v(4);
        s.psi   = v(5);
        int nCmg = (int)ref.deltas.size();
        s.deltas = v.segment(6, nCmg);
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

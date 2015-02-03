#ifndef PENGUIN_WRIST_HPP
#define PENGUIN_WRIST_HPP

#include <Eigen/Core>

class PenguinWrist {
  private:
    double _solve_up();
    double _solve_middle();
    double _solve_down();
    void _init_constants();
    double _deg2rad(double ang);
    void _solve();

  public:
    PenguinWrist();
    ~PenguinWrist();

    void set_rpy(double r, double p, double y);
    void set_rpy(Eigen::Vector3d);
    Eigen::Vector3d get_solution();

  private:
    /* Variables */

    double _A, _B, _C, _D, _E, _F, _H, _I, _L, _M, _N, _O, _P, _R;
    double _l1a, _l2a, _psi, _rl2;
    Eigen::Vector3d _centerXY, _p4;
    Eigen::Vector3d _RPY;
    Eigen::Vector3d _sols;
};

#endif

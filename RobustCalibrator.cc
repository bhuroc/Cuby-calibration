/**
 *@file RobustCalibrator.cc
 *@brief 
 */
#include <iostream>
#include <BVL/math/optim/SAMinimizer.h>
#include <BVL/math/optim/SimplexMinimizer.h>
#include <BVL/math/optim/deriv.h>
#include "RobustCalibrator.h"
#include "Robust.h"

using namespace std;
using namespace BVL;

void RobustCalibrator::calibrate(Calibrator *c)
{
    OptFunc f(c->left_top,c->left_bottom,c->right_top,c->right_bottom, c->top_plane,c->bottom_plane,c->screen_info);
    SimplexMinimizer<OptFunc> s(15, &f);
    //SAMinimizer<OptFunc> s(15, &f);
    s.params->maxevals=50000;
    s.params->eps = 1e-10;
    Vector<double> x_sol(15,c->viewer_parameters);
    s.run(x_sol.get_data());
    cerr << "opt exit info : " << s.get_exit_info() << "\n";
    s.get_details(cerr);

    gradient = grad(f, x_sol);

    double diff = 0.;
    for(int i=1; i<=15; ++i) {
	double d = calibrator->viewer_parameters[i-1]-x_sol(i);
	diff += d*d;
	calibrator->viewer_parameters[i-1] = x_sol(i);
#ifdef DEBUG
	cerr << calibrator->viewer_parameters[i-1] << " ";
#endif
    }

}

void RobustCalibrator::write_info(std::ostream &os)
{
    os << "Gradient at est:"<<gradient << "\n";
}



/**
 *@file SimpleOptim.cc
 *@brief 
 */
#include <iostream>
#include <BVL/math/stats/stats.h>
#include "Calibrator.h"
#include "PlaneFit.h"
#include "CameraLeastSquare.h"

using namespace BVL;
using namespace std;

Vector<double> plane_ray_intersection(const PlaneEquation &plane, const Vector<double> &o, const Vector<double> &d)
{
    // the ray is o+td
    double t = -(plane.normal(1)*o(1)+plane.normal(2)*o(2)+plane.normal(3)*o(3)+plane.distance)/(plane.normal(1)*d(1)+plane.normal(2)*d(2)+plane.normal(3)*d(3));
    return  o + d*t;
}

void Calibrator::simple_optimize()
{
    int num_points=0;
    num_points+=left_top_samples.size();
    num_points+=left_bottom_samples.size();
    num_points+=right_top_samples.size();
    num_points+=right_bottom_samples.size();

    CamFunc f(&left_camera, &right_camera, left_top_samples,left_bottom_samples,right_top_samples,right_bottom_samples, top_plane,bottom_plane,screen_info);
    BVL::NLSSolver<CamFunc, CamFunc> s2(3*num_points, 15, &f);

    s2.params->epsilon = 1e-10;
    s2.params->maxfev = 40000;

    int n_two_eye_params = 15;
    Vector<double> two_eye_params (n_two_eye_params);

    two_eye_params[0] = left_camera.get_alpha_u();
    two_eye_params[1] = left_camera.get_k22();
    two_eye_params[2] = right_camera.get_alpha_u();
    //two_eye_params[3] = right_camera.get_k22();
    two_eye_params[3] = left_camera.get_k12()/two_eye_params[0];
    two_eye_params[4] = left_camera.get_u0();
    two_eye_params[5] = left_camera.get_v0();
    two_eye_params[6] = right_camera.get_u0();
    two_eye_params[7] = right_camera.get_v0();

    Matrix<double> R=left_camera.get_rotation();
    Vector<double> euler = matrix2euler(R);
    two_eye_params[9] = euler(1);
    two_eye_params[10] = euler(2);
    two_eye_params[11] = euler(3);
    Vector<double> ttrans=left_camera.get_translation();
    two_eye_params[12] = ttrans[0];
    two_eye_params[13] = ttrans[1];
    two_eye_params[14] = ttrans[2];

    double mmpd = left_camera.get_pixel_size();
    two_eye_params[8]=mmpd;

    s2.run(two_eye_params.get_data());
    //s2.get_details();

    Vector<double> fx(3*num_points);
    f(fx.get_data(), two_eye_params.get_data());
    vector<double> total_error;
    cerr << "Final errors: ";
    for(int i=0; i<num_points;++i) {
	Vector<double> nm(3);
	nm(1) = fx[3*i];
	nm(2) = fx[3*i+1];
	nm(3) = fx[3*i+2];
	cerr << i << ": " << nm << "\t\t" << norm(nm) << "\n";
	total_error.push_back(norm(nm));
    }
    Stats<double> st(total_error);
    cerr << "** Average error " << st.get_mean() << " sd " << st.get_std() << "\n";

}



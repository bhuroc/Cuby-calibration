/**************************************************
*
* $Id$
*
**************************************************/
/**
 *@file EM.cc
 *@brief 
 */
#include <iostream>
#include <BVL/math/stats/stats.h>
#include "EM.h"

//#define NEWTON
//#define SIMPLEX
//#define LEASTSQUARES

#if defined(NEWTON)
    #include "Uncmin.h"
    #include "Newton.h"
#elif defined(SIMPLEX)
    #include <BVL/math/optim/SAMinimizer.h>
    #include <BVL/math/optim/SimplexMinimizer.h>
    #include "Newton.h"
#else
    #include "LeastSquares.h"
#endif

#define DEBUG

using namespace std;
using namespace BVL;


Matrix<double> inverse2x2(const Matrix<double> &m)
{
    Matrix<double> im(2,2);
    double det=m(1,1)*m(2,2)-m(1,2)*m(2,1);
    im(1,1) = m(2,2)/det;
    im(1,2) = -m(1,2)/det;
    im(2,1) = -m(2,1)/det;
    im(2,2) = m(1,1)/det;
    return im;
}

void EMCalibrator::calibrate(Calibrator *c) 
{
    calibrator = c;

    bool stop = false;
    int num_iters = 0;
    int samples = 0;
    while(!stop && num_iters < calibrator->max_num_iters) {

	EStep(samples);

	if( MStep() < calibrator->ftol)  
	    stop = true;

	++num_iters;
        if(num_iters < 10)
            samples = 5;
        else
            samples = 20;
    }
}

Vector<double> EMCalibrator::project_with_viewer_params(const Point5D &p, Calibrator::EyeId which_eye)
{
    /////////////////////
    // Do the projection using the viewer parameterizations
    /////////////////////

    // Transform the 3D point to the screen space 
    // Note r and t is the transformation from screen to the world
    // and we need to bring points from world to screen
    BVL::Matrix<double> r=BVL::euler2matrix(calibrator->viewer_parameters[6],calibrator->viewer_parameters[7],calibrator->viewer_parameters[8]);
    BVL::Vector<double> x_scr(3);
    x_scr(1) = p.x-calibrator->viewer_parameters[9]; 
    x_scr(2) = p.y-calibrator->viewer_parameters[10]; 
    x_scr(3) = p.z-calibrator->viewer_parameters[11];
    x_scr = transpose(r) * x_scr;

    // Get the intersection of the line of sight with the view plane z=0
    // Let e be the eye, the line of sight is 
    // e+\lambda (x_scr-e)
    // \lambda = -e_z/(x_scr_z - e_z)
    double lamb;
    double sx, sy; // intersection on the screen
    if(which_eye == Calibrator::LEFT_EYE) {
      lamb = - calibrator->viewer_parameters[2]/(x_scr[2]-calibrator->viewer_parameters[2]);
      sx = calibrator->viewer_parameters[0]+lamb*(x_scr[0]-calibrator->viewer_parameters[0]);
      sy = calibrator->viewer_parameters[1]+lamb*(x_scr[1]-calibrator->viewer_parameters[1]);
    }else {
      lamb = - calibrator->viewer_parameters[5]/(x_scr[2]-calibrator->viewer_parameters[5]);
      sx = calibrator->viewer_parameters[3]+lamb*(x_scr[0]-calibrator->viewer_parameters[3]);
      sy = calibrator->viewer_parameters[4]+lamb*(x_scr[1]-calibrator->viewer_parameters[4]);
    }

    // Transform sx, sy to pixels
    double ucenter = (calibrator->screen_info.width-1)/2.;
    double vcenter = (calibrator->screen_info.height-1)/2.;

    Vector<double> image_point(2);
    image_point(1) = sx/calibrator->viewer_parameters[14]+calibrator->viewer_parameters[13]*sy/calibrator->viewer_parameters[14]+ucenter;
    image_point(2) = vcenter - sy/(calibrator->viewer_parameters[14]*calibrator->viewer_parameters[12]);

#ifdef DEBUG
    cerr << p << " projected to " << image_point << "\n";
#endif
    return image_point;
}

/**
 * Sample from p(s_i|C^t, x_i)\propto p(x_i|s_i,C^t)p(s_i|C^t,f_i)
 */
void EMCalibrator::sample_screen_points(const Point5D &p, vector<Point5D> &samples, int n, MultiNormalGenerator &g, const Matrix<double> &cov_proj, Calibrator::EyeId which_eye)
{
    Point5D sample;
    //Vector<double> mean=project_with_viewer_params(p, which_eye);
    Vector<double> mean(Tuple<double>(2, p.sx, p.sy));

    g.set_parameters(mean, cov_proj);
    Vector<double> noise(2);
    for(int i=0; i<n; ++i) {
        noise = g.generate();
        sample.sx = noise(1);
        sample.sy = noise(2);
        sample.dfx = sample.sx - p.sx;
        sample.dfy = sample.sy - p.sy;
        sample.x = p.x;
        sample.y = p.y;
        sample.z = p.z;

        samples.push_back(sample);
        //cerr << noise << "|" << sample << "\n";
    }
}

Point5D EMCalibrator::sample_image_point(const Point5D &p, MultiNormalGenerator &g)
{
    Point5D sample;                                           
    Vector<double> n=g.generate();                            

    // n is Gaussian noise                                    
    sample.sx = p.sx + n(1);                                  
    sample.sy = p.sy + n(2);                                  
    sample.x = p.x;                                           
    sample.y = p.y;                                           
    sample.z = p.z;                                           

    return sample;                                            
}


void EMCalibrator::EStep(int n_samples)
{
    // for each fiducial point, generate multiple samples
    // and the sum of errors is the expectation.
    left_top_samples.clear();
    left_bottom_samples.clear();
    right_top_samples.clear();
    right_bottom_samples.clear();

    if(n_samples == 0) {
        copy(calibrator->left_top.begin(),calibrator->left_top.end(),back_inserter(left_top_samples));
        copy(calibrator->left_bottom.begin(),calibrator->left_bottom.end(),back_inserter(left_bottom_samples));
        copy(calibrator->right_top.begin(),calibrator->right_top.end(),back_inserter(right_top_samples));
        copy(calibrator->right_bottom.begin(),calibrator->right_bottom.end(),back_inserter(right_bottom_samples));
        return;
    }

    // the viewer parameters have been updated by now,  create
    // the new Gaussian distribution accordingly
    double sd_scr_points = calibrator->screen_info.fiducial_size/1.5;
    Matrix<double> cov_scr(2,2,Tuple<double>(2, sd_scr_points,sd_scr_points));
    Matrix<double> cov_projection(2,2,Tuple<double>(2, 1.35, 1.35));

    // because of the shear and aspect ratio, change the covariance matrix
    // with the inverse of the affine transformation
    Matrix<double> iaffine_trans(2,2,0.);

    double aspect_ratio = calibrator->viewer_parameters[12];
    double skew = calibrator->viewer_parameters[13];
    // {{1,s},{0,1/a}}^-1 = {{1,-as},{0,a}}
    iaffine_trans(1,1) = 1.;
    iaffine_trans(1,2) = -skew*aspect_ratio;
    iaffine_trans(2,2) = aspect_ratio;
    cov_scr = transpose(iaffine_trans)*cov_scr*iaffine_trans;
    //cov_projection = transpose(iaffine_trans)*cov_projection*iaffine_trans;

    MultiNormalGenerator g(Vector<double>(2,0.), cov_scr);
    //g.seed(rand());
#if THERESOFFSET
    for(size_t i=0;i<left_top.size(); ++i) {
        sample_screen_points(left_top[i], left_top_samples, n_samples, g, cov_projection, LEFT_EYE);
    }
    for(size_t i=0;i<left_bottom.size(); ++i) {
        sample_screen_points(left_bottom[i], left_bottom_samples, n_samples, g, cov_projection, LEFT_EYE);
    }
    for(size_t i=0;i<right_top.size(); ++i) {
        sample_screen_points(right_top[i], right_top_samples, n_samples, g, cov_projection, RIGHT_EYE);
    }
    for(size_t i=0;i<right_bottom.size(); ++i) {
        sample_screen_points(right_bottom[i], right_bottom_samples, n_samples, g, cov_projection, RIGHT_EYE);
    }
#else
    for(int n=0; n<n_samples; ++n) {
        for(size_t i=0;i<calibrator->left_top.size(); ++i) {
            left_top_samples.push_back(sample_image_point(calibrator->left_top[i], g));
        }
        for(size_t i=0;i<calibrator->left_bottom.size(); ++i) {
            left_bottom_samples.push_back(sample_image_point(calibrator->left_bottom[i], g));
        }
        for(size_t i=0;i<calibrator->right_top.size(); ++i) {
            right_top_samples.push_back(sample_image_point(calibrator->right_top[i], g));
        }
        for(size_t i=0;i<calibrator->right_bottom.size(); ++i) {
            right_bottom_samples.push_back(sample_image_point(calibrator->right_bottom[i], g));
        }
    }
#endif

}


double EMCalibrator::MStep()
{
#if defined(NEWTON)
    OptFunc f(left_top_samples,left_bottom_samples,right_top_samples,right_bottom_samples, top_plane,bottom_plane,screen_info);
    Uncmin<Vector<double>, Matrix<double>, OptFunc> min(&f);
    //min.SetPrint(stderr, 1, 1);
    min.SetMethod(1);
    Vector<double> scale(Tuple<double>(15, 30.,300.,500.,30.,300.,300.,500.,3.,3.,3.,100.,200.,3000.,1.,0.0001,0.3));
    if(min.SetScaleArg(scale)) {
	cerr << "Setting typical value failed.\n";
	return 0.;
    }

    min.SetScaleFunc(400);//typical value of total_error
    min.SetMaxIter(100);

    Vector<double> x(15, Calibrator::viewer_parameters),x_sol(15),g_sol(15);
    double f_sol;
    int info = min.Minimize(x,x_sol,f_sol,g_sol);
    int msg = min.GetMessage();
    cerr << "Message from optimization " << msg << "\n";
    cerr << "function at minimum " << f_sol << "\n";
    cerr << "gradient at minimum " << g_sol << "\n";

#elif defined(SIMPLEX) 

    OptFunc f(left_top_samples,left_bottom_samples,right_top_samples,right_bottom_samples, top_plane,bottom_plane,screen_info);
    SimplexMinimizer<OptFunc> s(15, &f);
    //SAMinimizer<OptFunc> s(15, &f);
    s.params->maxevals=50000;
    s.params->eps = 1e-10;
    Vector<double> x_sol(15,viewer_parameters);
    s.run(x_sol.get_data());

    s.get_details(cerr);

    /** 
     * for simulation: to compare the function value.
    cerr << "value at the true parameters:";
    cerr << f(simulation_parameters) << "\n";
    */
    
#else

    int num_points=0;
    num_points+=left_top_samples.size();
    num_points+=left_bottom_samples.size();
    num_points+=right_top_samples.size();
    num_points+=right_bottom_samples.size();
    Func f(left_top_samples,left_bottom_samples,right_top_samples,right_bottom_samples, calibrator->top_plane,calibrator->bottom_plane,calibrator->screen_info);
    BVL::NLSSolver<Func, Func> s2(3*num_points, 15, &f);

    s2.params->epsilon = 1e-10;
    s2.params->maxfev = 40000;

    Vector<double> x_sol(15,calibrator->viewer_parameters);
    s2.run(x_sol.get_data());
    //s2.get_details();

    Vector<double> fx(3*num_points);
    f(fx.get_data(), x_sol.get_data());
    vector<double> total_error;
    //cerr << "Final errors: ";
    for(int i=0; i<num_points;++i) {
	Vector<double> nm(3);
	nm(1) = fx[3*i];
	nm(2) = fx[3*i+1];
	nm(3) = fx[3*i+2];
	total_error.push_back(norm(nm));
    }
#ifdef DEBUG
    Stats<double> st(total_error);
    cerr << "** Average error " << st.get_mean() << " sd " << st.get_std() << "\n";
#endif

#endif

    // update the viewer parameters
#ifdef DEBUG
    cerr << "MStep done ";
#endif

    double diff = 0.;
    for(int i=1; i<=15; ++i) {
	double d = calibrator->viewer_parameters[i-1]-x_sol(i);
	diff += d*d;
	calibrator->viewer_parameters[i-1] = x_sol(i);
#ifdef DEBUG
	cerr << calibrator->viewer_parameters[i-1] << " ";
#endif
    }
#ifdef DEBUG
    cerr << "\n **** diff to previous iter "<< diff << "\n";
#endif

#if defined(NEWTON)
    return 1.;
#elif defined(LEASTSQUARES)
    return 0.;
#else
    return diff;
#endif
}


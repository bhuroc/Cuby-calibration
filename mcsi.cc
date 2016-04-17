/**
 *@file mcsi.cc
 *@brief 
 */
#include <iostream>
#include <iomanip>
#include <getopt.h>
#include <algorithm>
#include <numeric>
#include <time.h>
#include <VML/System/IniParser.h>
#include <BVL/math/stats/rng.h>
#include "ScreenPointGenerator.h"
#include "Calibrator.h"
#include "Point5D.h"

using namespace std;
using namespace BVL;
using namespace VML;

#define MIN(x,y) ((x)<(y)?(x):(y))

#define DEBUG
bool debug = false;

enum EyeId {LEFT_EYE, RIGHT_EYE};

// the set of true calibration parameters
// This is from calib_bh.txt where the iod is 5.5.  Recalibrated
// using iod prior of 6.3
double true_values[15]={
    -51.0053,-95.5572,484.971,
    7.34575,-101.629,507.564,
    -0.423412,1.53665,-0.527912,
    -154.036,163.873,-2871.1,
    1.00301,-0.00109048,0.338837
};

double noise2d, noise3d;
ScreenInfo screen_info;
MultiNormalGenerator screen_gen, noise3d_gen;
RandomNumberGenerator uniform;
PlaneEquation top_plane_world, bottom_plane_world;
double top_plane_dist, bottom_plane_dist;
double ucenter, vcenter;

vector<Point5D> left_top, left_bottom, right_top, right_bottom;

extern Matrix<double> inverse2x2(const Matrix<double> &m);

double meet3d(const Point5D &p, EyeId which_eye, double plane_dist)
{
    BVL::Matrix<double> r=BVL::euler2matrix(true_values[6],true_values[7],true_values[8]);
    BVL::Vector<double> t(3);
    t(1) = true_values[9]; 
    t(2) = true_values[10]; 
    t(3) = true_values[11];
    double aspect_ratio = true_values[12];
    double skew = true_values[13];
    double pixelsize = true_values[14];

    // transform to screen coordinates
    double sy=pixelsize * (p.sy-vcenter)/aspect_ratio;
    double sx=pixelsize * (p.sx-ucenter) -skew*sy;
    BVL::Vector<double> s(BVL::Tuple<double>(3, sx,-sy,0.));
    BVL::Vector<double> cop(3);
    if(which_eye == LEFT_EYE)
        cop = Tuple<double>(3, true_values[0],true_values[1],true_values[2]);
    else
        cop = Tuple<double>(3, true_values[3],true_values[4],true_values[5]);

    PlaneEquation plane;
    plane.normal = Vector<double>(3, "0 0 1");
    plane.distance = plane_dist;

    double cd=dot_prod(cop,plane.normal);
    double denom=cd - dot_prod(s,plane.normal);
    Vector<double> dp(3);
    cd += plane.distance;
    dp(1)=cop(1)+cd*(s(1)-cop(1))/denom;
    dp(2)=cop(2)+cd*(s(2)-cop(2))/denom;
    dp(3)=cop(3)+cd*(s(3)-cop(3))/denom;


    // transform it to the world
    dp = r*dp+t;
    dp(1)-=p.x;
    dp(2)-=p.y;
    dp(3)-=p.z;

    return norm_squared(dp);
}

void sample_3d(Point5D &p, EyeId which_eye, double plane_dist)
{
    BVL::Matrix<double> r=BVL::euler2matrix(true_values[6],true_values[7],true_values[8]);
    BVL::Vector<double> t(3);
    t(1) = true_values[9]; 
    t(2) = true_values[10]; 
    t(3) = true_values[11];
    double aspect_ratio = true_values[12];
    double skew = true_values[13];
    double pixelsize = true_values[14];

    // sample a fiducial point
    Vector<double> ps(Tuple<double>(2, p.sx,p.sy));
    if(noise2d!=0) {
        Vector<double> psn=screen_gen.generate();
        ps += psn;
    }

    // transform to screen coordinates
    double sy=pixelsize * (ps(2)-vcenter)/aspect_ratio;
    double sx=pixelsize * (ps(1)-ucenter) -skew*sy;
    BVL::Vector<double> s(BVL::Tuple<double>(3, sx,-sy,0.));
    BVL::Vector<double> cop(3);
    if(which_eye == LEFT_EYE)
        cop = Tuple<double>(3, true_values[0],true_values[1],true_values[2]);
    else
        cop = Tuple<double>(3, true_values[3],true_values[4],true_values[5]);

    PlaneEquation plane;
    plane.normal = Vector<double>(3, "0 0 1");
    plane.distance = plane_dist;

    double cd=dot_prod(cop,plane.normal);
    double denom=cd - dot_prod(s,plane.normal);
    Vector<double> dp(3);
    cd += plane.distance;
    dp(1)=cop(1)+cd*(s(1)-cop(1))/denom;
    dp(2)=cop(2)+cd*(s(2)-cop(2))/denom;
    dp(3)=cop(3)+cd*(s(3)-cop(3))/denom;

    // add noise
    if(noise3d != 0) {
        Vector<double> dpn=noise3d_gen.generate();
        dp += dpn;
        if(debug) {
            cerr << "noise " << dpn << "\n";
        }
    }

    // transform it to the world
    dp = r*dp+t;

    p.x = dp(1);
    p.y = dp(2);
    p.z = dp(3);

}

void generate_3D_points(vector<Point5D> &p, EyeId eye, double plane_dist)
{
    for(int i=0; i<p.size(); ++i) {
        sample_3d(p[i], eye, plane_dist);
        if(debug) {
            cerr << i << " " << p[i] << "\n";
        }
    }

}

void help(const char *p)
{
    cerr << "Usage "<<p<<" <options>\n";
    cerr << "--screenwidth (-w): width of the screen\n";
    cerr << "--screenheight (-h): height of the screen\n";
    cerr << "--fiducialsize (-f): size of the fiducial\n";
    cerr << "--numfiducials (-n): number of fiducial points on each pass\n";
    cerr << "--nummcruns (-r): number of monte carlo runs\n";
    cerr << "--noise2d (-s): noise on the screen \n";
    cerr << "--noise3d (-x): noise in 3D space\n";
    cerr << "--usecenteronly (-e): don't do EM, use the center of fiducial points.\n";
    cerr << "--proposalsize(-p): size of step of the proposal distribution.\n";
    cerr << "--test (-t): test the result by looking the residuals.\n";
    cerr << "--help (-?): this message\n";
}


int num_accepted = 0;
double proposal_size;

Vector<double> proposal_dist(const Vector<double> &s)
{
    Matrix<double> A(2,2,0.);
    A(1,1) = 1;
    A(1,2) = true_values[13];
    A(2,1) = true_values[12];
    Matrix<double> cov(2,2,0.);
    cov(1,1) = proposal_size;
    cov(2,2) = proposal_size;

    //cov=transpose(A)*cov*A;
    screen_gen.set_parameters(s, cov);
    return screen_gen.generate();
}

double log_target_dist(const Point5D &p, const Point5D &fi)
{
    double d1 = meet3d(p, LEFT_EYE, top_plane_dist);

    Matrix<double> A(2,2,0.);
    A(1,1) = 1;
    A(1,2) = true_values[13];
    A(2,1) = true_values[12];
    Matrix<double> cov(2,2,0.);
    cov(1,1) = 1./4;
    cov(2,2) = 1./4;
    //cov=inverse2x2(transpose(A)*cov*A);

    double dfx=p.sx-fi.sx;
    double dfy=p.sy-fi.sy;
    double d2=cov(1,1)*dfx*dfx+cov(2,2)*dfy*dfy+2.*cov(1,2)*dfx*dfy;

    //cerr << "p " << p << " fi " << fi << "\n";
    //cerr << "*** d1 " << d1 << " d2 " << d2 << "\n";
    return d1+d2;
}


Vector<double> mcmc(const Vector<double> &s, const Point5D &ref)
{
    double u=uniform.generate();
    Vector<double> sstar=proposal_dist(s);
    Point5D pstar, p;
    pstar.x = ref.x;
    pstar.y = ref.y;
    pstar.z = ref.z;
    pstar.sx = sstar(1);
    pstar.sy = sstar(2);

    p.x = ref.x;
    p.y = ref.y;
    p.z = ref.z;
    p.sx = s(1);
    p.sy = s(2);

    double ratio=exp(
            MIN(-log_target_dist(pstar, ref)+log_target_dist(p, ref),0.));
    if(u<ratio) {
        num_accepted ++;
        return sstar;
    }else
        return s;
}

vector<Vector<double> > samples;
void calc_mean_cov()
{
    Vector<double> m=accumulate(samples.begin(), samples.end(), Vector<double>(2,0.));

    m /= samples.size();

    cout << "mean " << m<< "\n";
    for(int i=0; i<2; ++i) {
        for(int j=0; j<2; ++j) {
            double qij = 0.;
            for(int k=0; k<samples.size(); ++k) {
                qij += (samples[k][i]-m[i])*(samples[k][j]-m[j]);
            }
            cout << qij/samples.size() << " ";
        }
        cout << "\n";
    }

}

void sample_si(const Point5D &p, int n)
{
    cerr << "input point " << p << "\n";

    // start with p itself
    Vector<double> s0(2);
    s0(1) = p.sx;
    s0(2) = p.sy;

    // get a sample from the proposal distribution

    // run the sampler for this many times
    const int burnin=100;
    for(int i=0; i<burnin;++i) {
        s0 = mcmc(s0, p);
    }
    //cerr << "-----------------\n";

    for(int i=0; i<n; ++i) {
        s0 = mcmc(s0, p);
        //cout << s0 << "\n";
        samples.push_back(s0);
    }

    cerr << "accept rate: " << num_accepted/double(burnin+n)<< "\n";
}


int main(int argc, char **argv)
{
    if(argc < 2) {
        help(argv[0]);
        return 1;
    }

    // config items
    // screen width 1152
    // screen height 864
    // fiducial size 6
    // noise in 3D space 1mm (calc'ed from real data)
    // number of fiducial points
    // number of MC runs
    static struct option opts[]={
        {"sreenwidth", required_argument, NULL, 'w'},
        {"screenheight", required_argument, NULL, 'h'},
        {"fiducialsize", required_argument, NULL, 'f'},
        {"numfiducials", required_argument, NULL, 'n'},
        {"nummcruns", required_argument, NULL, 'r'},
        {"noise2d", required_argument, NULL, 's'},
        {"noise3d", required_argument, NULL, 'x'},
        {"usecenteronly", no_argument, NULL, 'c'},
        {"test", no_argument, NULL, 't'},
        {"proposalsize", required_argument, NULL, 'p'},
        {"help", no_argument, NULL, '?'},
        {NULL, 0, NULL, 0}
    };
    int ch;
    int screen_width=1152,screen_height=864;
    int fiducial_size=6;
    int num_points=0;
    int num_rounds=100;
    noise3d = 1.;
    noise2d = 3.;
    bool use_center_only = false;
    bool test_residuals = false;
    while((ch = getopt_long(argc, argv, "w:h:f:n:r:p:s:x:tc?", opts, NULL))!=-1) {
        switch(ch) {
            case 'w':
                screen_width = atoi(optarg);
                break;
            case 'h':
                screen_height = atoi(optarg);
                break;
            case 'f':
                fiducial_size = atoi(optarg);
                break;
            case 'n':
                num_points = atoi(optarg);
                break;
            case 'r':
                num_rounds = atoi(optarg);
                break;
            case 'p':
                proposal_size = atof(optarg);
                break;
            case 's':
                noise2d = atof(optarg);
                break;
            case 'x':
                noise3d = atof(optarg);
                break;
            case 'c':
                use_center_only = true;
                break;
            case 't':
                test_residuals = true;
                break;
            case '?':
            default:
                help(argv[0]);
                return 0;
        }
    }


    screen_info.width=screen_width;
    screen_info.height=screen_height;
    screen_info.fiducial_size = fiducial_size;
    ucenter = (screen_info.width-1)/2.;
    vcenter = (screen_info.height-1)/2.;

    Calibrator calib(screen_info, .05, 10.);

    // noise generator
    Matrix<double> cov_scr(2,2,Tuple<double>(2, noise2d,noise2d));
    if(noise2d != 0)
        screen_gen.set_parameters(Vector<double>(2,0.), cov_scr);

    if(noise3d != 0) 
        noise3d_gen.set_parameters(Vector<double>(3,0.), Matrix<double>(3,3,Tuple<double>(3, noise3d, noise3d, noise3d/2)));

    uniform.set_distribution("uniform", 0., 1.);
    // create the screen points, margin is set to 50 pixels
    ScreenPointGenerator gen(screen_width, screen_height, num_points, 50, true);
    vector<double> screen_x = gen.get_x_points();
    vector<double> screen_y = gen.get_y_points();
    cerr << "exact number of points " << gen.get_num_points() << "\n";
    for(int i=0; i<num_points; ++i) {
        Point5D p;
        p.sx = screen_x.at(i);
        p.sy = screen_y.at(i);
        left_top.push_back(p);
        left_bottom.push_back(p);
        right_top.push_back(p);
        right_bottom.push_back(p);
    }

    // distance of the two planes in viewer space
    top_plane_dist = -81.;
    bottom_plane_dist = 10.;
    BVL::Matrix<double> R_scr2world=BVL::euler2matrix(true_values[6],true_values[7],true_values[8]);
    BVL::Vector<double> t(3);
    t(1) = true_values[9]; 
    t(2) = true_values[10]; 
    t(3) = true_values[11];
    // get the true plane equations in the world
    BVL::Vector<double> normal_scr(3, "0 0 1");
    BVL::Vector<double> normal_world=R_scr2world*normal_scr;
    double d_top_world = top_plane_dist - dot_prod(normal_world, t);
    double d_bottom_world = bottom_plane_dist - dot_prod(normal_world, t);
    top_plane_world.normal = normal_world;
    top_plane_world.distance = d_top_world;
    bottom_plane_world.normal= normal_world;
    bottom_plane_world.distance = d_bottom_world;
    cerr << "top plane in world " << top_plane_world << "\n";
    cerr << "bottom plane in world " << bottom_plane_world << "\n";

    // seed the noise generators
    screen_gen.seed(rand());
    noise3d_gen.seed(rand());

    // generate the 3D points 
    generate_3D_points(left_top, LEFT_EYE, top_plane_dist);
    generate_3D_points(left_bottom, LEFT_EYE, bottom_plane_dist);
    generate_3D_points(right_top, RIGHT_EYE, top_plane_dist);
    generate_3D_points(right_bottom, RIGHT_EYE, bottom_plane_dist);

    for(int i=0; i<num_points; ++i)  {
        samples.clear();
        sample_si(left_top[i], num_rounds);
        calc_mean_cov();
    }
        
    return 0;
}




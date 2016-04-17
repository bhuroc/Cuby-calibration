/**
 *@file simulate.cc
 *@brief 
 */
#include <iostream>
#include <iomanip>
#include <getopt.h>
#include <algorithm>
#include <numeric>
#include <string>
#include <iterator>
#include <time.h>
#include <VML/System/IniParser.h>
#include <BVL/math/stats/rng.h>
#include "ScreenPointGenerator.h"
#include "Calibrator.h"
#include "Point5D.h"

using namespace std;
using namespace BVL;
using namespace VML;

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
int num_outliers;
ScreenInfo screen_info;
MultiNormalGenerator screen_gen, noise3d_gen, outlier_gen;
PlaneEquation top_plane_world, bottom_plane_world;

vector<Point5D> left_top, left_bottom, right_top, right_bottom;

void output_raw_data(const string &file)
{
    ofstream os(file.c_str());
    copy(left_top.begin(), left_top.end(), ostream_iterator<Point5D>(os, "\n"));
    copy(left_bottom.begin(), left_bottom.end(), ostream_iterator<Point5D>(os, "\n"));
    copy(right_top.begin(), right_top.end(), ostream_iterator<Point5D>(os, "\n"));
    copy(left_bottom.begin(), left_bottom.end(), ostream_iterator<Point5D>(os, "\n"));
}

void meet3d(Point5D &p, EyeId which_eye, double plane_dist)
{
    double ucenter = (screen_info.width-1)/2.;
    double vcenter = (screen_info.height-1)/2.;
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
        //cout << "screen noise " << psn << " " << ps << "\n";
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

    if(debug) {
        cerr << "dp before " << dp << "\t";
    }

    // transform it to the world
    dp = r*dp+t;

    // add noise
    if(noise3d != 0) {
        Vector<double> dpn=noise3d_gen.generate();
        if(num_outliers > 0) {
            dpn = outlier_gen.generate();
            num_outliers --;
        }
        dp += dpn;
        debug = true;
        if(debug) {
            cerr << "noise " << dpn << "\n";
        }
        debug = false;
    }

    p.x = dp(1);
    p.y = dp(2);
    p.z = dp(3);

}

void generate_3D_points(vector<Point5D> &p, EyeId eye, double plane_dist)
{
    for(size_t i=0; i<p.size(); ++i) {
        meet3d(p[i], eye, plane_dist);
        if(debug) {
            cerr << i << " " << p[i] << "\n";
        }
    }

}

void compute_mean_sd(const vector<Vector<double> > &data, Vector<double> &mean, Vector<double> &sd)
{
    mean = accumulate(data.begin(), data.end(), Vector<double>(15, 0.));
    mean /= data.size();

    sd = 0.;
    for(size_t i=0; i<data.size(); ++i) {
        for(int j=0; j<15; ++j) {
            sd[j]=+(data[i][j]-mean[j])*(data[i][j]-mean[j]);
        }
    }
    sd = sd/(data.size()-1.);
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
    cerr << "--test (-t): test the result by looking the residuals.\n";
    cerr << "--samplescreenonce (-o): use the same screen points for multiple simulation.\n";
    cerr << "--rawdata (-d): save raw data to specified file.\n";
    cerr << "--bad (-b): how many outliers to use.\n";
    cerr << "--help (-?): this message\n";
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
        {"samplescreenonce", no_argument, NULL, 'o'},
        {"rawdata", required_argument, NULL, 'd'},
        {"bad",required_argument, NULL, 'b'},
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
    bool sample_screen_once = false;
    string raw_data_file;
    num_outliers = 0;
    while((ch = getopt_long(argc, argv, "w:h:f:n:r:s:x:tcod:b:?", opts, NULL))!=-1) {
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
            case 'o':
                sample_screen_once = true;
                break;
            case 'd':
                raw_data_file = optarg;
                break;
            case 'b':
                num_outliers = atoi(optarg);
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

    Calibrator calib(screen_info, .05, 10.);
    calib.set_simulation_parameters(true_values);

    vector<Vector<double> > results;

    // noise generator
    Matrix<double> cov_scr(2,2,Tuple<double>(2, noise2d,noise2d));
    /* this is not necessary, since when transforming from pixels to
     * the screen, the skew and apsect ratio have been applied.
    Matrix<double> iaffine_trans(2,2,0.);
    double aspect_ratio = true_values[12];
    double skew = true_values[13];
    iaffine_trans(1,1) = 1.;
    iaffine_trans(1,2) = -skew/aspect_ratio;
    iaffine_trans(2,2) = 1./aspect_ratio;
    cov_scr = transpose(iaffine_trans)*cov_scr*iaffine_trans;
    */
    if(noise2d != 0)
        screen_gen.set_parameters(Vector<double>(2,0.), cov_scr);

    if(noise3d != 0) 
        noise3d_gen.set_parameters(Vector<double>(3,0.), Matrix<double>(3,3,Tuple<double>(3, noise3d, noise3d, noise3d)));
    if(num_outliers > 0 ) {
        outlier_gen.set_parameters(Vector<double>(3,10.), Matrix<double>(3,3,Tuple<double>(3, 50.*noise3d, 50.*noise3d, 50.*noise3d)));
    }

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
    double top_plane_dist, bottom_plane_dist;
    top_plane_dist = -81.;
    bottom_plane_dist = -10.;
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

    //srand(time(NULL));

    ////////////////////////////////////////////////////////////////////////
    // run the calibration for many times
    ////////////////////////////////////////////////////////////////////////
    for(int i=0; i<num_rounds; ++i) {

        // seed the noise generators
        if(sample_screen_once)
            screen_gen.seed(13);
        else
            screen_gen.seed(i+rand());
        noise3d_gen.seed(rand());

        // generate the 3D points 
        generate_3D_points(left_top, LEFT_EYE, top_plane_dist);
        generate_3D_points(left_bottom, LEFT_EYE, bottom_plane_dist);
        generate_3D_points(right_top, RIGHT_EYE, top_plane_dist);
        generate_3D_points(right_bottom, RIGHT_EYE, bottom_plane_dist);

        if(raw_data_file.size()) {
            output_raw_data(raw_data_file);
        }

        // calibrate
        if(use_center_only)
            calib.calibrate(left_top, left_bottom, right_top, right_bottom, 1);
        else
            calib.calibrate(left_top, left_bottom, right_top, right_bottom, 51);

        // store the calibration results
        results.push_back(Vector<double>(15, calib.viewer_parameters));
        cerr <<i <<"\t" << results.back() << "\n";
    }
    if(debug)
        calib.write_result(cout);

    // get the statistics of the results.
    Vector<double> mean(15), sd(15);
    compute_mean_sd(results, mean, sd);
    cout <<setw(5)<<std::left<< "i"<<setw(15)<<std::right<<"mean"<<setw(15)<<std::right<<"diff"<<setw(15)<<std::right<<"sd\n";
    for(int i=0; i<15;++i) {
        cout << std::setprecision(6)<<setw(5)<<std::left<< i+1<<setw(15)<<std::right << mean[i] <<setw(15)<<std::right<<mean[i]-true_values[i] << setw(15)<<std::right <<sd[i]<<"\n"; 
    }

    if(test_residuals) {
        calib.test_by_meet3d();
        cerr << "\n------------------ true values ---------------------\n";
        calib.test_with_given(Vector<double>(15,true_values));
        if(sample_screen_once)
            calib.test_by_reprojection();
    }

    return 0;
}


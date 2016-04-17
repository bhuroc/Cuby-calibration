/**
 *@file simulate.cc
 *@brief 
 */
#include "Calibrator.h"
#include "Point5D.h"
#include "ScreenPointGenerator.h"
#include <BVL/math/stats/rng.h>
#include <VML/System/IniParser.h>
#include <algorithm>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <time.h>

using namespace std;
using namespace BVL;
using namespace VML;
#define DEBUG
bool debug = false;

enum EyeId { LEFT_EYE,
    RIGHT_EYE };

// the set of true calibration parameters
// This is from calib_bh.txt where the iod is 5.5.  Recalibrated
// using iod prior of 6.3
double true_values[15] = {
    -51.0053, -95.5572, 484.971,
    7.34575, -101.629, 507.564,
    -0.423412, 1.53665, -0.527912,
    -154.036, 163.873, -2871.1,
    1.00301, -0.00109048, 0.338837
};

double noise2d, noise3d;
ScreenInfo screen_info;
MultiNormalGenerator screen_gen, noise3d_gen;
PlaneEquation top_plane_world, bottom_plane_world;

vector<Point5D> left_top, left_bottom, right_top, right_bottom;

void meet3d(Point5D& p, EyeId which_eye, double plane_dist)
{
    double ucenter = (screen_info.width - 1) / 2.;
    double vcenter = (screen_info.height - 1) / 2.;
    BVL::Matrix<double> r = BVL::euler2matrix(true_values[6], true_values[7], true_values[8]);
    BVL::Vector<double> t(3);
    t(1) = true_values[9];
    t(2) = true_values[10];
    t(3) = true_values[11];
    double aspect_ratio = true_values[12];
    double skew = true_values[13];
    double pixelsize = true_values[14];

    // sample a fiducial point
    Vector<double> ps(Tuple<double>(2, p.sx, p.sy));
    if (noise2d != 0) {
        Vector<double> psn = screen_gen.generate();
        ps += psn;
#ifdef DEBUG2
        cerr << "screen noise " << psn << " " << ps << "\n";
#endif
    }

    // transform to screen coordinates
    double sy = pixelsize * (ps(2) - vcenter) / aspect_ratio;
    double sx = pixelsize * (ps(1) - ucenter) - skew * sy;
    BVL::Vector<double> s(BVL::Tuple<double>(3, sx, -sy, 0.));
    BVL::Vector<double> cop(3);
    if (which_eye == LEFT_EYE)
        cop = Tuple<double>(3, true_values[0], true_values[1], true_values[2]);
    else
        cop = Tuple<double>(3, true_values[3], true_values[4], true_values[5]);

    PlaneEquation plane;
    plane.normal = Vector<double>(3, "0 0 1");
    plane.distance = plane_dist;

    double cd = dot_prod(cop, plane.normal);
    double denom = cd - dot_prod(s, plane.normal);
    Vector<double> dp(3);
    cd += plane.distance;
    dp(1) = cop(1) + cd * (s(1) - cop(1)) / denom;
    dp(2) = cop(2) + cd * (s(2) - cop(2)) / denom;
    dp(3) = cop(3) + cd * (s(3) - cop(3)) / denom;

    if (debug) {
        cerr << "dp before " << dp << "\t";
    }
    // add noise
    if (noise3d != 0) {
        Vector<double> dpn = noise3d_gen.generate();
        dp += dpn;
        if (debug) {
            cerr << "noise " << dpn << "\n";
        }
    }

    // transform it to the world
    dp = r * dp + t;

    p.x = dp(1);
    p.y = dp(2);
    p.z = dp(3);
#ifdef DEBUG2
    cerr << "aspect " << aspect_ratio << " skew " << skew << " pixel " << pixelsize << "\n";
    cerr << "sx " << sx << " sy " << sy << " cd " << cd << "\n";
    cerr << "p " << p << "\n";
#endif
}

void generate_3D_points(vector<Point5D>& p, EyeId eye, double plane_dist)
{
    for (int i = 0; i < p.size(); ++i) {
        meet3d(p[i], eye, plane_dist);
        if (debug) {
            cerr << i << " " << p[i] << "\n";
        }
    }
}

void help(const char* p)
{
    cerr << "Usage " << p << " <options>\n";
    cerr << "--screenwidth (-w): width of the screen\n";
    cerr << "--screenheight (-h): height of the screen\n";
    cerr << "--fiducialsize (-f): size of the fiducial\n";
    cerr << "--numfiducials (-n): number of fiducial points on each pass\n";
    cerr << "--nummcruns (-r): number of monte carlo runs\n";
    cerr << "--noise2d (-s): noise on the screen \n";
    cerr << "--noise3d (-x): noise in 3D space\n";
    cerr << "--usecenteronly (-e): don't do EM, use the center of fiducial points.\n";
    cerr << "--test (-t): test the result by looking the residuals.\n";
    cerr << "--help (-?): this message\n";
}

Vector<double> project_to_screen(const double* viewer_parameters, const Point5D& p, EyeId which_eye)
{
    /////////////////////
    // Do the projection using the viewer parameterizations
    /////////////////////

    // Transform the 3D point to the screen space
    // Note r and t is the transformation from screen to the world
    // and we need to bring points from world to screen
    BVL::Matrix<double> r = BVL::euler2matrix(viewer_parameters[6], viewer_parameters[7], viewer_parameters[8]);
#ifdef DEBUG
    cerr << std::setprecision(10) << "source point " << p << "\n";
    cerr << "euler\n"
         << viewer_parameters[6] << " " << viewer_parameters[7] << " " << viewer_parameters[8] << "\n";
    cerr << "rotation\n"
         << r << "translation\n"
         << viewer_parameters[9] << " " << viewer_parameters[10] << " " << viewer_parameters[11] << "\n";
#endif
    BVL::Vector<double> x_scr(3);
    x_scr(1) = p.x - viewer_parameters[9];
    x_scr(2) = p.y - viewer_parameters[10];
    x_scr(3) = p.z - viewer_parameters[11];
    x_scr = transpose(r) * x_scr;
#ifdef DEBUG
    cerr << "point in screen " << x_scr << "\n";
#endif

    // Get the intersection of the line of sight with the view plane z=0
    // Let e be the eye, the line of sight is
    // e+\lambda (x_scr-e)
    // \lambda = -e_z/(x_scr_z - e_z)
    double lamb;
    double sx, sy; // intersection on the screen
    if (which_eye == LEFT_EYE) {
        lamb = -viewer_parameters[2] / (x_scr[2] - viewer_parameters[2]);
        sx = viewer_parameters[0] + lamb * (x_scr[0] - viewer_parameters[0]);
        sy = viewer_parameters[1] + lamb * (x_scr[1] - viewer_parameters[1]);
    } else {
        lamb = -viewer_parameters[5] / (x_scr[2] - viewer_parameters[5]);
        sx = viewer_parameters[3] + lamb * (x_scr[0] - viewer_parameters[3]);
        sy = viewer_parameters[4] + lamb * (x_scr[1] - viewer_parameters[4]);
    }

    sy = -sy;
#ifdef DEBUG
    cerr << "sensor coords " << sx << " " << sy << "\n";
#endif
    // Transform sx, sy to pixels
    double ucenter = (screen_info.width - 1) / 2.;
    double vcenter = (screen_info.height - 1) / 2.;

    Vector<double> image_point(2);
    image_point(1) = sx / viewer_parameters[14] + viewer_parameters[13] * sy / viewer_parameters[14] + ucenter;
    image_point(2) = vcenter + sy / (viewer_parameters[14] * viewer_parameters[12]);

#ifdef DEBUG
    cerr << p << " projected to " << image_point << "\n\n";
#endif
    image_point(1) = sx;
    image_point(2) = sy;
    return image_point;
}

void sample_si(const Point5D& p, int n)
{
    cerr << "input point " << p << "\n";

    // set the rng's mean to p.x,y,z
    Vector<double> m(Tuple<double>(3, p.x, p.y, p.z));
    Matrix<double> cov(3, 3, Tuple<double>(3, .5, .5, .5)); // .5mm

    noise3d_gen.set_parameters(m, cov);
    for (int i = 0; i < n; ++i) {
        // generate a sample of x_i
        Vector<double> xi = noise3d_gen.generate();
        Point5D x;
        x.x = xi(1);
        x.y = xi(2);
        x.z = xi(3);

        // compute s_i
        Vector<double> s_i = project_to_screen(true_values, x, LEFT_EYE);
        cout << xi << " " << s_i << "\n";
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
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
    static struct option opts[] = {
        { "sreenwidth", required_argument, NULL, 'w' },
        { "screenheight", required_argument, NULL, 'h' },
        { "fiducialsize", required_argument, NULL, 'f' },
        { "numfiducials", required_argument, NULL, 'n' },
        { "nummcruns", required_argument, NULL, 'r' },
        { "noise2d", required_argument, NULL, 's' },
        { "noise3d", required_argument, NULL, 'x' },
        { "usecenteronly", no_argument, NULL, 'c' },
        { "test", no_argument, NULL, 't' },
        { "help", no_argument, NULL, '?' },
        { NULL, 0, NULL, 0 }
    };
    int ch;
    int screen_width = 1152, screen_height = 864;
    int fiducial_size = 6;
    int num_points = 0;
    int num_rounds = 100;
    noise3d = 1.;
    noise2d = 3.;
    bool use_center_only = false;
    bool test_residuals = false;
    while ((ch = getopt_long(argc, argv, "w:h:f:n:r:s:x:tc?", opts, NULL)) != -1) {
        switch (ch) {
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
        case '?':
        default:
            help(argv[0]);
            return 0;
        }
    }

    screen_info.width = screen_width;
    screen_info.height = screen_height;
    screen_info.fiducial_size = fiducial_size;

    Calibrator calib(screen_info, .05, 10.);

    // noise generator
    Matrix<double> cov_scr(2, 2, Tuple<double>(2, noise2d, noise2d));
    if (noise2d != 0)
        screen_gen.set_parameters(Vector<double>(2, 0.), cov_scr);

    if (noise3d != 0)
        noise3d_gen.set_parameters(Vector<double>(3, 0.), Matrix<double>(3, 3, Tuple<double>(3, noise3d, noise3d, noise3d / 2)));

    // create the screen points, margin is set to 50 pixels
    ScreenPointGenerator gen(screen_width, screen_height, num_points, 50, true);
    vector<double> screen_x = gen.get_x_points();
    vector<double> screen_y = gen.get_y_points();
    cerr << "exact number of points " << gen.get_num_points() << "\n";
    for (int i = 0; i < num_points; ++i) {
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
    bottom_plane_dist = 100.;
    BVL::Matrix<double> R_scr2world = BVL::euler2matrix(true_values[6], true_values[7], true_values[8]);
    BVL::Vector<double> t(3);
    t(1) = true_values[9];
    t(2) = true_values[10];
    t(3) = true_values[11];
    // get the true plane equations in the world
    BVL::Vector<double> normal_scr(3, "0 0 1");
    BVL::Vector<double> normal_world = R_scr2world * normal_scr;
    double d_top_world = top_plane_dist - dot_prod(normal_world, t);
    double d_bottom_world = bottom_plane_dist - dot_prod(normal_world, t);
    top_plane_world.normal = normal_world;
    top_plane_world.distance = d_top_world;
    bottom_plane_world.normal = normal_world;
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

    sample_si(left_bottom[5], num_rounds);

    return 0;
}


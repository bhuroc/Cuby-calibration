/**
 *@file Calibrator.cc
 *@brief 
 */
#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include "Calibrator.h"
#include "PlaneFit.h"
#include <BVL/camera/CameraCalibrator.h>
#include <BVL/camera/CameraOptimizer.h>
#include <BVL/math/stats/stats.h>
#include <math.h>

// There are for test_with_given()
#if defined(NEWTON)
#include "Newton.h"
#include "Uncmin.h"
#elif defined(SIMPLEX)
#include "Newton.h"
#include <BVL/math/optim/SAMinimizer.h>
#include <BVL/math/optim/SimplexMinimizer.h>
#else
#include "LeastSquares.h"
#endif

#define DEBUG

/**
 * The term skew and aspect ratio, when used in viewer_parameters, are not
 * exactly accurate.  Skew is the ratio between k12/k11 and aspect ratio
 * is k11/k22.  We know there are 3 DOFs and the top 2x2 of the intrinsic
 * matrix is
 * 1/p   s/p
 * 0    1/(ap)
 *
 * So s is really ctan(\theta), where \theta is the skew angle.  a is more
 * confusing.  Written in this way, it looks like the aspect ratio: pixel
 * size on the u direction is p and on the v direciton is ap.  But it's really
 * A\sin\theta, where A is the aspect ratio.
 *
 * It didn't help the paper uses the reall meaning of skew and aspect ratio
 * and we also need to transform between this convenient representation to
 * and from the camera class.
 */
using namespace std;
using namespace BVL;

double Calibrator::viewer_parameters[17];
double Calibrator::simulation_parameters[17];
void print_x()
{
    for (int i = 0; i < 15; ++i)
        cerr << Calibrator::viewer_parameters[i] << " ";
    cerr << "\n";
}

Calibrator::Calibrator(const ScreenInfo& s, double tolerance, double table_offset)
    : screen_info(s)
    , ftol(tolerance)
    , table_bottom_offset(table_offset)
    , left_eye_pos(3, 0.)
    , right_eye_pos(3, 0.)
{
    viewer = 0;
    nonlinear_calibrator = 0;
}

Calibrator::~Calibrator()
{
    if (viewer)
        delete viewer;
}

Stats<double> stats_row, stats_col;

void Calibrator::reproject(const PerspectiveCamera& camera, const vector<Point5D>& points, ostream& os)
{
    for (size_t i = 0; i < points.size(); ++i) {
        double row, col;
        camera.project(points[i].x, points[i].y, points[i].z, row, col);
        os << col << " " << row << " -- " << col - points[i].sx << " " << row - points[i].sy << " : " << points[i] << "\n";
        stats_row.add_data(row - points[i].sy);
        stats_col.add_data(col - points[i].sx);
    }
}

Stats<double> stats3dx, stats3dy, stats3dz;

double Calibrator::meet3d(const Point5D& p, const PlaneEquation& pl, EyeId which_eye, ostream& os)
{
    double ucenter = (screen_info.width - 1) / 2.;
    double vcenter = (screen_info.height - 1) / 2.;
    BVL::Matrix<double> r = BVL::euler2matrix(viewer_parameters[6], viewer_parameters[7], viewer_parameters[8]);
    BVL::Vector<double> t(3);
    t(1) = viewer_parameters[9];
    t(2) = viewer_parameters[10];
    t(3) = viewer_parameters[11];
    double aspect_ratio = viewer_parameters[12];
    double skew = viewer_parameters[13];
    double pixelsize = viewer_parameters[14];
    double sy = pixelsize * (p.sy - vcenter) * aspect_ratio;
    double sx = pixelsize * (p.sx - ucenter) - skew * sy;
    BVL::Vector<double> s(BVL::Tuple<double>(3, sx, -sy, 0.));
    BVL::Vector<double> cop(3);
    if (which_eye == LEFT_EYE)
        cop = Tuple<double>(3, viewer_parameters[0], viewer_parameters[1], viewer_parameters[2]);
    else
        cop = Tuple<double>(3, viewer_parameters[3], viewer_parameters[4], viewer_parameters[5]);
    PlaneEquation plane;
    plane.normal = transpose(r) * pl.normal;
    plane.distance = dot_prod(t, pl.normal) + pl.distance;

    double cd = dot_prod(cop, plane.normal);
    double denom = cd - dot_prod(s, plane.normal);
    Vector<double> dp(3);
    cd += plane.distance;
    dp(1) = cop(1) + cd * (s(1) - cop(1)) / denom;
    dp(2) = cop(2) + cd * (s(2) - cop(2)) / denom;
    dp(3) = cop(3) + cd * (s(3) - cop(3)) / denom;
    dp = r * dp + t;
    Vector<double> x(Tuple<double>(3, p.x, p.y, p.z));
    x = dp - x;
    os << x << " -- " << norm(x) << "\n";
    stats3dx.add_data(x(1));
    stats3dy.add_data(x(2));
    stats3dz.add_data(x(3));

    return norm(x);
}

void Calibrator::test_by_meet3d()
{
    vector<double> err;
    cout << "left top\n";
    for (size_t i = 0; i < left_top.size(); ++i) {
        cout << i << " " << left_top[i] << ":";
        err.push_back(meet3d(left_top[i], top_plane, LEFT_EYE, cout));
    }
    cout << "left bottom\n";
    for (size_t i = 0; i < left_bottom.size(); ++i) {
        cout << i << " " << left_bottom[i] << ":";
        err.push_back(meet3d(left_bottom[i], bottom_plane, LEFT_EYE, cout));
    }
    cout << "right top\n";
    for (size_t i = 0; i < right_top.size(); ++i) {
        cout << i << " " << right_top[i] << ":";
        err.push_back(meet3d(right_top[i], top_plane, RIGHT_EYE, cout));
    }
    cout << "right bottom\n";
    for (size_t i = 0; i < right_bottom.size(); ++i) {
        cout << i << " " << right_bottom[i] << ":";
        err.push_back(meet3d(right_bottom[i], bottom_plane, RIGHT_EYE, cout));
    }
    cerr << "error x\t error y\t error z\n";
    cerr << stats3dx.get_mean() << "\t";
    cerr << stats3dy.get_mean() << "\t";
    cerr << stats3dz.get_mean() << "\n";
    cerr << stats3dx.get_std() << "\t";
    cerr << stats3dy.get_std() << "\t";
    cerr << stats3dz.get_std() << "\n";
    cerr << "total error " << accumulate(err.begin(), err.end(), 0.) << "\n";
}

void Calibrator::test_by_reprojection()
{
    cout << "left top\n";
    reproject(left_camera, left_top, cout);
    cout << "left bottom\n";
    reproject(left_camera, left_bottom, cout);
    cout << "right top\n";
    reproject(right_camera, right_top, cout);
    cout << "right bottom\n";
    reproject(right_camera, right_bottom, cout);

    cout << "Error stats: \n";
    cout << "Row " << stats_row.get_mean() << " " << stats_row.get_std() << "\n";
    cout << "Col " << stats_col.get_mean() << " " << stats_col.get_std() << "\n";
}

void Calibrator::compute_viewer()
{
    ////////////////////////////////////////
    //
    // Compute the pixel size of the screen
    //
    ////////////////////////////////////////

    // the eye positions in the world coord
    Vector<double> left_camera_pos = left_camera.get_location();
    Vector<double> right_camera_pos = right_camera.get_location();

    Vector<double> eyediff = left_camera_pos - right_camera_pos;
    double eye_dist = norm(eyediff);
#ifdef DEBUG
    cerr << "interocular dist " << eye_dist << "\n";
#endif
    eyediff = eyediff / eye_dist;

    // transform the vector to camera's coord
    Matrix<double> rotation = left_camera.get_rotation();
    eyediff = rotation * eyediff;

    // angle between eyediff and the normal of the imaging plane
    double angle = acos(eyediff(3));
#ifdef DEBUG
    cerr << "eye screen angle is " << angle * 180. / M_PI << "\n";
#endif

    // project the distance to the imaging plane
    eye_dist = eye_dist * sin(angle);

    // Because we know the aspect ratio,
    // we have enough information to compute the pixel size of both
    // axes.  Let the pixel size of the u axis is p_u, we have
    // (\delta v * p_v)^2+(\delta u * p_u)^2 = real_dist^2.
    // p_v = \beta p_u, \beta is the aspect ratio, \beta=k11/k22
    double delta_u = left_camera.get_u0() - right_camera.get_u0();
    double delta_v = left_camera.get_v0() - right_camera.get_v0();
    double aspect_ratio = left_camera.get_alpha_u() / left_camera.get_k22();
    if (aspect_ratio > 1.3 || aspect_ratio < 0.7) {
        aspect_ratio = 1.;
#ifdef DEBUG
        cerr << "aspect ratio is forced to be 1.\n";
#endif
    }
    double theta = atan(1. / left_camera.get_skew());
    double scr_dist = sqrt(delta_u * delta_u + aspect_ratio * aspect_ratio * delta_v * delta_v + 2 * delta_u * delta_v * aspect_ratio * cos(theta));

#ifdef DEBUG
    cerr << "aspect_ratio " << aspect_ratio << " or in angles " << theta * 180 / M_PI << ". The cross term in screen dist " << 2 * delta_u * delta_v * aspect_ratio * cos(theta) << "\n";
    cerr << "screen dist " << scr_dist << "\n";
#endif

    double mmpd = eye_dist / scr_dist;
    left_camera.set_pixel_size(mmpd);
    right_camera.set_pixel_size(mmpd);
#ifdef DEBUG
    cerr << "pixel size " << mmpd << "\n";
#endif
    // the dot pitch shouldn't be two far from 0.3
    if (fabs(mmpd - 0.3) > 0.1) {
#ifdef DEBUG
        cerr << "force mmpd to be 0.3\n";
#endif
        mmpd = 0.3;
    }

    ////////////////////////////////////////
    //
    // Construct the viewer representation
    //
    ////////////////////////////////////////
    double ucenter = (screen_info.width - 1) / 2.;
    double vcenter = (screen_info.height - 1) / 2.;
    // Notice the y is flipped.  In the screen coord, y goes from bottom
    // to top, as opposed in the camera coord.
    // Also to calcuate x, which uses y*skew, y is the pre-flipped value,
    // commanded by the intrinsic matrix math.
    double skew = left_camera.get_k12() / left_camera.get_alpha_u();
    left_eye_pos(2) = mmpd * (left_camera.get_v0() - vcenter) * aspect_ratio;
    left_eye_pos(1) = mmpd * (left_camera.get_u0() - ucenter) - skew * left_eye_pos(2);
    left_eye_pos(3) = left_camera.get_focal_length();
    left_eye_pos(2) = -left_eye_pos(2);

    skew = right_camera.get_k12() / right_camera.get_alpha_u();
    aspect_ratio = right_camera.get_alpha_u() / right_camera.get_k22();
    right_eye_pos(2) = mmpd * (right_camera.get_v0() - vcenter) * aspect_ratio;
    right_eye_pos(1) = mmpd * (right_camera.get_u0() - ucenter) - skew * right_eye_pos(2);
    right_eye_pos(3) = right_camera.get_focal_length();
    right_eye_pos(2) = -right_eye_pos(2);

#ifdef DEBUG
    cerr << "left eye position in scr " << left_eye_pos << "\n";
    cerr << "right eye position in scr " << right_eye_pos << "\n";
#endif

    // Rotation from the screen coord to the world coord
    rotation_scr2world = transpose(rotation);
    // The y and z of the screen coord are different from the camera
    // coord, so we need to negate those
    for (int i = 1; i <= 3; ++i)
        for (int j = 2; j <= 3; ++j)
            rotation_scr2world(i, j) = -rotation_scr2world(i, j);
#ifdef DEBUG
    cerr << "rotation from screen to world\n"
         << rotation_scr2world;
#endif

    translation_scr2world = -transpose(rotation) * left_camera.get_translation() - rotation_scr2world * left_eye_pos;
#ifdef DEBUG
    cerr << "translation from screen to world " << translation_scr2world << "\n";
#endif
}

void Calibrator::write_raw_data(std::ostream& os, vector<Point5D>::const_iterator b, vector<Point5D>::const_iterator e)
{
    int n = 1;
    vector<Point5D>::const_iterator it = b;
    while (it != e)
        os << n++ << ":" << setprecision(15) << *it++ << "\n";
}

void Calibrator::normal2angles(const Vector<double>& n, double& elevation, double& azimuth)
{
    assert(fabs(norm(n) - 1.) < 1e-10);

    elevation = asin(n(3));
    azimuth = atan2(n(2), n(1));
}

void Calibrator::write_result(std::ostream& os)
{
    os << "[Lefteye]\n";
    os << "parameters:";
    left_camera.save_camera_parameters(os);
    os << "\n";
    os << "[Righteye]\n";
    os << "parameters:";
    right_camera.save_camera_parameters(os);
    os << "\n";

    os << "[Screen]\n";
    os << "width:" << screen_info.width << "\n";
    os << "height:" << screen_info.height << "\n";
    os << "fiducial size:" << screen_info.fiducial_size << "\n";
    os << "mmpd:" << left_camera.get_pixel_size() << "\n";

    os << "[Tabletop]\n";
    os << "# In world coordinate system\n";
    PlaneEquation table_eqn = bottom_plane;
    table_eqn.distance = bottom_plane.distance + table_bottom_offset;
    os << "equation:" << table_eqn << "\n";
    os << "table tool offset:" << table_bottom_offset << "\n";

    double* x = viewer_parameters;
    Vector<double> left(Tuple<double>(3, x[0], x[1], x[2]));
    Vector<double> right(Tuple<double>(3, x[3], x[4], x[5]));
    Matrix<double> rotation = euler2matrix(x[6], x[7], x[8]);
    Vector<double> trans(Tuple<double>(3, x[9], x[10], x[11]));
    double aspect_ratio = x[12];
    double skew = x[13];
    double pixel_size = x[14];
    os << "[Summary]\n";
    os << "interocular distance:" << norm(left - right) << "\n";
    os << "left eye in screen:" << left << "\n";
    os << "right eye in screen:" << right << "\n";
    os << "screen center in world:" << trans << "\n";
    os << "aspect ratio of pixels:" << aspect_ratio << "\n";
    os << "skew of pixels (deg):" << atan(-1. / skew) * 180. / M_PI << "\n";
    os << "screen size (width and height in mm):" << screen_info.width * pixel_size << " " << screen_info.height * pixel_size * aspect_ratio << "\n";

    info_s << "interocular distance:" << norm(left - right) << "\n";
    info_s << "screen center in world:" << trans << "\n";

    // rotation^T * bottom_plane.normal transforms the normal
    // of table top to the screen coord.  Its z component gives
    // the angle between the table and the screen.
    // NEW: show the full picture: elevation and azimuth, not just
    // the angle between the two normals
    Vector<double> normal_s = normalize(transpose(rotation) * bottom_plane.normal);
    double ele, azi;
    normal2angles(normal_s, ele, azi);
    // 90- because we'd be more interested in the angle between, not
    // the angle with the ground plane
    os << "table normal in screen, angle between and azimuth (deg):" << 90. - ele * 180. / M_PI << " " << azi * 180. / M_PI << "\n";

    // the angle between the screen normal and center-eye-center-of-screen
    // line
    Vector<double> center_eye = normalize(left + right);
    normal2angles(center_eye, ele, azi);
    os << "center eye in screen, angle between and azimuth (deg):" << 90. - ele * 180. / M_PI << " " << azi * 180. / M_PI << "\n";

    // plane equations in the viewer's coord
    PlaneEquation p1, p2, p3;
    p1.normal = transpose(rotation) * top_plane.normal;
    p1.distance = dot_prod(trans, top_plane.normal) + top_plane.distance;
    p2.normal = transpose(rotation) * bottom_plane.normal;
    p2.distance = dot_prod(trans, bottom_plane.normal) + bottom_plane.distance;
    os << "bottom plane in world:" << bottom_plane << "\n";
    os << "bottom plane in screen:" << p2 << "\n";
    os << "top plane in world:" << top_plane << "\n";
    os << "top plane in screen:" << p1 << "\n";
    p3 = p2;
    p3.distance = p2.distance + table_bottom_offset;
    os << "table in screen: " << p3 << "\n";

    info_s << "table in screen: " << p3 << "\n";

    // Give the optimizer a change to say something.
    nonlinear_calibrator->write_info(os);

    os << "[raw data left top]\n";
    write_raw_data(os, left_top.begin(), left_top.end());
    os << "[raw data left bottom]\n";
    write_raw_data(os, left_bottom.begin(), left_bottom.end());
    os << "[raw data right top]\n";
    write_raw_data(os, right_top.begin(), right_top.end());
    os << "[raw data right bottom]\n";
    write_raw_data(os, right_bottom.begin(), right_bottom.end());
}

void Calibrator::fit_world_planes()
{
    PlaneFit bottom, top;
    for (size_t i = 0; i < left_top.size(); ++i)
        top.add_point(left_top[i].x, left_top[i].y, left_top[i].z);
    for (size_t i = 0; i < right_top.size(); ++i)
        top.add_point(right_top[i].x, right_top[i].y, right_top[i].z);
    for (size_t i = 0; i < left_bottom.size(); ++i)
        bottom.add_point(left_bottom[i].x, left_bottom[i].y, left_bottom[i].z);
    for (size_t i = 0; i < right_bottom.size(); ++i)
        bottom.add_point(right_bottom[i].x, right_bottom[i].y, right_bottom[i].z);

    top_plane = top.get_plane_equation();
    bottom_plane = bottom.get_plane_equation();
#ifdef DEBUG
    cerr << "top plane " << top_plane << "\n";
    cerr << "top plane residuals\n";
    copy(top.get_residuals().begin(), top.get_residuals().end(), ostream_iterator<double>(cerr, " "));
    cerr << "\nbottom plane " << bottom_plane << "\n";
    cerr << "bottom plane residuals\n";
    copy(bottom.get_residuals().begin(), bottom.get_residuals().end(), ostream_iterator<double>(cerr, " "));
    cerr << "\n";
#endif
}

void Calibrator::linear_calibrate()
{
    CameraCalibrator left, right;
    for (size_t i = 0; i < left_top.size(); ++i)
        left.add_point(left_top[i].x, left_top[i].y, left_top[i].z, left_top[i].sx, left_top[i].sy);
    for (size_t i = 0; i < left_bottom.size(); ++i)
        left.add_point(left_bottom[i].x, left_bottom[i].y, left_bottom[i].z, left_bottom[i].sx, left_bottom[i].sy);
    for (size_t i = 0; i < right_top.size(); ++i)
        right.add_point(right_top[i].x, right_top[i].y, right_top[i].z, right_top[i].sx, right_top[i].sy);
    for (size_t i = 0; i < right_bottom.size(); ++i)
        right.add_point(right_bottom[i].x, right_bottom[i].y, right_bottom[i].z, right_bottom[i].sx, right_bottom[i].sy);

    left.calibrate();
    right.calibrate();
    left_camera = left.get_camera();
    right_camera = right.get_camera();
    left_camera.set_linear();
    right_camera.set_linear();

/*
    CameraImagePlaneOptimizer left_op(&left);
    CameraImagePlaneOptimizer right_op(&right);
    left.set_optimizer(&left_op);
    right.set_optimizer(&right_op);
    */

#ifdef DEBUG
    cerr << "linear calibration result:\n";
    cerr << "left eye\n"
         << left_camera << "\n";
    Matrix<double> left_image_res = left.compute_image_residuals();
    cerr << "left image residuals\n"
         << left_image_res << "\n";
    Matrix<double> right_image_res = right.compute_image_residuals();
    cerr << "right eye\n"
         << right_camera << "\n";
    cerr << "right image residuals\n"
         << right_image_res << "\n";
#endif

//left.optimize();
//right.optimize();

#ifdef DEBUG_OPTIM
    cerr << "After optimization\n";
    cerr << "left eye\n"
         << left_camera << "\n";
    left_image_res = left.compute_image_residuals();
    cerr << "left image residuals\n"
         << left_image_res << "\n";
    right_image_res = right.compute_image_residuals();
    cerr << "right eye\n"
         << right_camera << "\n";
    cerr << "right image residuals\n"
         << right_image_res << "\n";
#endif
}

void Calibrator::set_camera_parameters()
{
    double ucenter = (screen_info.width - 1) / 2.;
    double vcenter = (screen_info.height - 1) / 2.;

    // 0-2 left eye position in scr
    // 3-5 right eye position in scr
    // 6-8 rotation from scr to world
    // 9-11 translation from scr to world
    // 12 aspect ratio
    // 13 skew
    // 14 pixel size
    double* x = viewer_parameters;
    Vector<double> left(Tuple<double>(3, x[0], x[1], x[2]));
    Vector<double> right(Tuple<double>(3, x[3], x[4], x[5]));
    Matrix<double> rotation = euler2matrix(x[6], x[7], x[8]);
    Vector<double> trans(Tuple<double>(3, x[9], x[10], x[11]));
    double aspect_ratio = x[12];
    double skew = x[13];
    double pixel_size = x[14];
    Matrix<double> intrinsic(3, 3, 0.);
    intrinsic(1, 1) = left(3) / pixel_size;
    intrinsic(1, 2) = intrinsic(1, 1) * skew;
    intrinsic(2, 2) = intrinsic(1, 1) / aspect_ratio;
    // old version, don't know where the - comes from
    // Well, read the write up, see Eq. 8.  rev 12 corrected sign, from
    // + to - and now at rev 118, I went back to the wrong thing again.
    // Read the write up.
    intrinsic(1, 3) = left(1) / pixel_size - skew * left(2) / pixel_size + ucenter;
    intrinsic(2, 3) = vcenter - left(2) / (pixel_size * aspect_ratio);
    intrinsic(3, 3) = 1.;
    left_camera.set_intrinsic(intrinsic);
    left_camera.set_pixel_size(pixel_size);

    // the rigid body transformation
    // x_w = R_{s2w} x_s + t_{s2w}
    // but
    // x_s = diag(1,-1,-1) x_l + l
    // so we have
    // R_left = (R_{s2w} diag(1,-1,-1))^T and
    // t_left = - R_left (t_{s2w} + R_{s2w}  l)
    Matrix<double> F(3, 3, Tuple<double>(3, 1., -1., -1.));
    Matrix<double> R_w2eye = transpose(rotation * F);
    left_camera.set_rotation(R_w2eye);
    left_camera.set_translation(-R_w2eye * (trans + rotation * left));

    intrinsic(1, 1) = right(3) / pixel_size;
    intrinsic(1, 2) = intrinsic(1, 1) * skew;
    intrinsic(2, 2) = intrinsic(1, 1) / aspect_ratio;
    intrinsic(1, 3) = right(1) / pixel_size - skew * right(2) / pixel_size + ucenter;
    intrinsic(2, 3) = vcenter - right(2) / (pixel_size * aspect_ratio);
    right_camera.set_intrinsic(intrinsic);
    right_camera.set_pixel_size(pixel_size);
    right_camera.set_rotation(R_w2eye);
    right_camera.set_translation(-R_w2eye * (trans + rotation * right));
}

void Calibrator::create_viewer()
{
    // create the viewer
    viewer = new VML::ViewingTransforms(left_camera, right_camera, screen_info.width, screen_info.height);

    // The sign of the normal of the recorvered bottom plane is arbitrary and
    // we need to rectify it so that we can apply the offset correctly.  We
    // use the fact that the eyes are above the planes.
    double l_table_dist;
    Vector<double> lpos = left_camera.get_location();
    l_table_dist = dot_prod(lpos, bottom_plane.normal) + bottom_plane.distance;
    if (l_table_dist < 0) {
        bottom_plane.normal = -bottom_plane.normal;
        bottom_plane.distance = -bottom_plane.distance;
    }
    double table[4];
    table[0] = bottom_plane.normal(1);
    table[1] = bottom_plane.normal(2);
    table[2] = bottom_plane.normal(3);
    // table[3] = bottom_plane.distance - table_bottom_offset is wrong
    // think about moving the plane z-d=0 to z-d/2=0, we add d/2
    // instead of subtract.
    table[3] = bottom_plane.distance + table_bottom_offset;
    viewer->setup_tabletop(table);
}

void Calibrator::calibrate(const vector<Point5D>& lt, const vector<Point5D>& lb, const vector<Point5D>& rt, const vector<Point5D>& rb, int n_iters)
{
    max_num_iters = n_iters;

    // save the raw data
    left_top = lt;
    left_bottom = lb;
    right_top = rt;
    right_bottom = rb;

    // fit the two planes
    fit_world_planes();

    // find the initial guesses of the two eyes
    linear_calibrate();

    // turn the camera representation to the viewer representation:
    // the eye positions are expressed in the screen coord.
    compute_viewer();

    // pass the viewer parameters to the optimizer
    prepare_optimization();

    nonlinear_calibrator->calibrate(this);

    //XXX error bounds of the estimated parameters

    // post processing: transform the viewer representation to
    // camera presentation
    set_camera_parameters();

    create_viewer();
}

void Calibrator::prepare_optimization()
{
    // transfer viewer parameters to the optimizer
    Calibrator::viewer_parameters[0] = left_eye_pos(1);
    Calibrator::viewer_parameters[1] = left_eye_pos(2);
    Calibrator::viewer_parameters[2] = left_eye_pos(3);
    Calibrator::viewer_parameters[3] = right_eye_pos(1);
    Calibrator::viewer_parameters[4] = right_eye_pos(2);
    Calibrator::viewer_parameters[5] = right_eye_pos(3);
    Vector<double> euler = matrix2euler(rotation_scr2world);
    Calibrator::viewer_parameters[6] = euler(1);
    Calibrator::viewer_parameters[7] = euler(2);
    Calibrator::viewer_parameters[8] = euler(3);
    Calibrator::viewer_parameters[9] = translation_scr2world(1);
    Calibrator::viewer_parameters[10] = translation_scr2world(2);
    Calibrator::viewer_parameters[11] = translation_scr2world(3);
    Calibrator::viewer_parameters[12] = left_camera.get_alpha_u() / left_camera.get_k22();
    Calibrator::viewer_parameters[13] = left_camera.get_k12() / left_camera.get_alpha_u();
    Calibrator::viewer_parameters[14] = left_camera.get_pixel_size();

#ifdef DEBUG
    print_x();
#endif
}

void Calibrator::test_with_given(const Vector<double>& known)
{
    for (int i = 0; i < 15; ++i) {
        viewer_parameters[i] = known[i];
        cerr << i << " : " << viewer_parameters[i] << "\n";
    }
    test_by_meet3d();
    return;

#if !(defined(NEWTON) || defined(SIMPLEX))
    int num_points = 0;
    num_points += left_top.size();
    num_points += left_bottom.size();
    num_points += right_top.size();
    num_points += right_bottom.size();
    Func f(left_top, left_bottom, right_top, right_bottom, top_plane, bottom_plane, screen_info);

    Vector<double> fx(3 * num_points);
    f(fx.get_data(), viewer_parameters);
    vector<double> total_error;
    //cerr << "Final errors: ";
    for (int i = 0; i < num_points; ++i) {
        Vector<double> nm(3);
        nm(1) = fx[3 * i];
        nm(2) = fx[3 * i + 1];
        nm(3) = fx[3 * i + 2];
        cerr << i << ": " << nm << "\t\t" << norm(nm) << "\n";
        total_error.push_back(norm(nm));
    }
    Stats<double> st(total_error);
    cerr << "** Average error " << st.get_mean() << " sd " << st.get_std() << "\n";
#endif
}

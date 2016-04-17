#ifndef _CALIBRATOR_H_
#define _CALIBRATOR_H_

/**
 *@file Calibrator.h
 *@brief 
 */
#include <vector>
#include <iostream>
#include <sstream>
#include <BVL/math/linalg/linalg.h>
#include <BVL/math/stats/rng.h>
#include <BVL/camera/PerspectiveCamera.h>
#include <VML/Graphics/ViewingTransforms.h>
#include "NonlinearCalibrator.h"
#include "PlaneFit.h"
#include "Point5D.h"

struct ScreenInfo 
{
    ScreenInfo(int w, int h, int f):width(w),height(h),fiducial_size(f) {}
    ScreenInfo() {}
    int width, height; // size of the screen in pixels
    int fiducial_size; // size of the fiducial points in pixels
};

class Calibrator
{
    friend class EMCalibrator;
    friend class RobustCalibrator;

    public:
        enum EyeId {LEFT_EYE, RIGHT_EYE};

	// tolerance is the different of the viewparameters between two
	// iterations.
	// table_offset is the offset between the tip of the probe, when
	// it's tracing the bottom plane, and the table.  It equals to
	// the size of the cube.
	Calibrator(const ScreenInfo &s, double tolerance, double table_offset);
	~Calibrator();

	void calibrate(const std::vector<Point5D> &left_top, const std::vector<Point5D> &left_bottom, const std::vector<Point5D> &right_top, const std::vector<Point5D> &right_bottom, int n_iters=51);

        void set_nonlinear_calibrator(NonlinearCalibrator *c) {
            nonlinear_calibrator = c;
        }

	void write_result(std::ostream &os);

	const VML::ViewingTransforms *get_viewer() const {
            return viewer;
        }

        std::string get_calibration_info() const {
            return info_s.str();
        }

        void reproject(const BVL::PerspectiveCamera &camera, const std::vector<Point5D> &points, std::ostream &os);
        void test_by_reprojection();
        void test_with_given(const BVL::Vector<double> &known);
        double meet3d(const Point5D &p, const PlaneEquation &pl, EyeId which, std::ostream &os);
        void test_by_meet3d();

        // 15 for the camera parameters, 2 for the robust formulation.
        static double viewer_parameters[17];
        static double simulation_parameters[17];

        void set_simulation_parameters(const double *s) {
            for(int i=0; i<17; ++i) {
                simulation_parameters[i] = s[i];
            }
        }

    private:
        void fit_world_planes();
        void linear_calibrate();
        void write_raw_data(std::ostream &os,std::vector<Point5D>::const_iterator b, std::vector<Point5D>::const_iterator e);
        void prepare_optimization();
        void compute_viewer();
        void set_camera_parameters();
        void create_viewer();

        void normal2angles(const BVL::Vector<double> &n, double &elevation, double &azimuth);

        NonlinearCalibrator *nonlinear_calibrator;
        ScreenInfo screen_info;
        double ftol;
        double table_bottom_offset;
        int max_num_iters;
        PlaneEquation top_plane, bottom_plane;
        BVL::PerspectiveCamera left_camera, right_camera;

        // we need to save the raw data for output and optimization
        std::vector<Point5D> left_top,left_bottom,right_top, right_bottom;

	// viewer representation
	BVL::Vector<double> left_eye_pos, right_eye_pos;
	BVL::Matrix<double> rotation_scr2world;
	BVL::Vector<double> translation_scr2world;

	VML::ViewingTransforms *viewer;

        // info to outside
        std::stringstream info_s;
};


#endif/*_CALIBRATOR_H_*/


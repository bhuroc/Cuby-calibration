#ifndef _EM_H_
#define _EM_H_

/**
 *@file EM.h
 *@brief 
 */
#include <vector>
#include <BVL/math/linalg/linalg.h>
#include <BVL/math/stats/rng.h>
#include "Point5D.h"
#include "NonlinearCalibrator.h"
#include "Calibrator.h"

class EMCalibrator : public NonlinearCalibrator
{
    public:
        virtual ~EMCalibrator() {}
        void calibrate(Calibrator *c);
    private:
        BVL::Vector<double> EMCalibrator::project_with_viewer_params(const Point5D &p, Calibrator::EyeId which_eye);
        void sample_screen_points(const Point5D &p, std::vector<Point5D> &samples, int n, BVL::MultiNormalGenerator &g, const BVL::Matrix<double> &cov_proj, Calibrator::EyeId which_eye);

        Point5D sample_image_point(const Point5D &p, BVL::MultiNormalGenerator &g);

        void EStep(int n_samples);
        double MStep();

	std::vector<Point5D> left_top_samples, left_bottom_samples;
	std::vector<Point5D> right_top_samples, right_bottom_samples;

        Calibrator *calibrator;
};

#endif/*_EM_H_*/


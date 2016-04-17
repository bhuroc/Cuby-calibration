#ifndef _CAMERALEASTSQUARE_H_
#define _CAMERALEASTSQUARE_H_

/**
 *@file CameraLeastSquare.h
 *@brief 
 */
#include "PlaneFit.h"
#include "Point5D.h"
#include <BVL/camera/PerspectiveCamera.h>
#include <BVL/math/linalg/linalg.h>
#include <BVL/math/nlssolver/nlssolver.h>
#include <iostream>
#include <vector>

extern BVL::Vector<double> plane_ray_intersection(const PlaneEquation& plane, const BVL::Vector<double>& o, const BVL::Vector<double>& d);

class CamFunc {

    public:
        CamFunc(BVL::PerspectiveCamera* l, BVL::PerspectiveCamera* r, const std::vector<Point5D>& lt, const std::vector<Point5D>& lb, const std::vector<Point5D>& rt, const std::vector<Point5D>& rb, const PlaneEquation& p1, const PlaneEquation& p2, const ScreenInfo& s)
            : left(l)
              , right(r)
              , left_top_samples(lt)
              , left_bottom_samples(lb)
              , right_top_samples(rt)
              , right_bottom_samples(rb)
              , top_plane(p1)
              , bottom_plane(p2)
    {
    }

        void error(double* fx, const BVL::PerspectiveCamera* cam, const std::vector<Point5D>& points, const PlaneEquation& plane, int& nf) const
        {
            BVL::Vector<double> loc = cam->get_location();
            BVL::Vector<double> image_direct(3);
            BVL::Matrix<double> RT = transpose(cam->get_rotation());

            double nx, ny;
            for (size_t i = 0; i < points.size(); ++i) {
                cam->image_coord_to_normalized_coord(points[i].sx, points[i].sy, nx, ny);
                image_direct(1) = nx;
                image_direct(2) = ny;
                image_direct(3) = 1.;
                image_direct = normalize(image_direct);

                // change it to world coord
                image_direct = RT * image_direct;

                // the line loc+image_direct*t intersects with object plane
                BVL::Vector<double> ip;
                ip = plane_ray_intersection(plane, loc, image_direct);

                fx[nf++] = ip(1) - points[i].x;
                fx[nf++] = ip(2) - points[i].y;
                fx[nf++] = ip(3) - points[i].z;
            }
        }

        // the layout of x:
        // 0: au_left,
        // 1: k22_left,
        // 2: au_right,
        // 3: skew,
        // 4: u0_left,
        // 5: v0_left,
        // 6: u0_right,
        // 7: v0_right,
        // 8: pixel_size,
        // 9-11: rx, ry, rz,
        // 12-14: tx_left,ty_left,tz_left.
        // tx_right,ty_right,tz_right are computed from u0 and v0
        //
        void operator()(double* fx, double* x) const
        {

            // update camera parameters using x
            left->set_alpha_u(x[0]);
            left->set_k22(x[1]);
            right->set_alpha_u(x[2]);

            // compute k22 of right,given au,k22 of left (=> aspect ratio)
            // and au of the right
            left->set_k12(x[3] * x[0]);
            right->set_k12(x[3] * x[2]);
            double left_alpha_v = left->get_alpha_v();
            double a_ratio = left_alpha_v / x[0];
            double right_alpha_v = x[2] * a_ratio;
            double ctg_theta = -x[3];
            double sin_theta = sqrt(1 / (1 + ctg_theta * ctg_theta));
            double right_k22 = right_alpha_v / sin_theta;
            right->set_k22(right_k22);

            left->set_u0(x[4]);
            left->set_v0(x[5]);
            right->set_u0(x[6]);
            right->set_v0(x[7]);

            left->set_pixel_size(x[8]);
            right->set_pixel_size(x[8]);

            BVL::Matrix<double> R = BVL::euler2matrix(x[9], x[10], x[11]);
            left->set_rotation(R);
            right->set_rotation(R);
            left->set_translation(x[12], x[13], x[14]);

            //compute translation of the right eye from
            // u0, v0 of the right eye
            // X_l is the right eye's position in left eye's coords
            // X_l = (0,0,0)^T-t_r+t_l
            double xl1, xl2, xl3;
            double tr1, tr2, tr3;
            left->image_coord_to_sensor_coord(x[6], x[7], xl1, xl2);
            xl3 = left->get_focal_length() - right->get_focal_length();
            tr1 = x[12] - xl1;
            tr2 = x[13] - xl2;
            tr3 = x[14] - xl3;
            right->set_translation(tr1, tr2, tr3);

            // minimize the error on the object planes
            int nf = 0;
            error(fx, left, left_top_samples, top_plane, nf);
            error(fx, left, left_bottom_samples, bottom_plane, nf);
            error(fx, right, right_top_samples, top_plane, nf);
            error(fx, right, right_bottom_samples, bottom_plane, nf);
        }

        BVL::PerspectiveCamera *left, *right;
        const std::vector<Point5D> &left_top_samples, &left_bottom_samples;
        const std::vector<Point5D> &right_top_samples, &right_bottom_samples;
        PlaneEquation top_plane, bottom_plane;
};

#endif /*_CAMERALEASTSQUARE_H_*/


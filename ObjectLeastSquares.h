#ifndef _OBJECTLEASTSQUARES_H_
#define _OBJECTLEASTSQUARES_H_

/**
 *@file ObjectLeastSquares.h
 *@brief 
 */

#include "Point5D.h"
#include <BVL/camera/PerspectiveCamera.h>
#include <BVL/math/linalg/linalg.h>
#include <BVL/math/nlssolver/nlssolver.h>
#include <iostream>
#include <vector>

class Func {
public:
    Func(const std::vector<Point5D>& lt, const std::vector<Point5D>& lb, const std::vector<Point5D>& rt, const std::vector<Point5D>& rb, const PlaneEquation& p1, const PlaneEquation& p2, const ScreenInfo& s)
        : left_top_samples(lt)
        , left_bottom_samples(lb)
        , right_top_samples(rt)
        , right_bottom_samples(rb)
        , top_plane(p1)
        , bottom_plane(p2)
    {
        ucenter = (s.width - 1) / 2.;
        vcenter = (s.height - 1) / 2.;
        counter = 0;
        num_points = left_top_samples.size() + left_bottom_samples.size() + right_top_samples.size() + right_bottom_samples.size();
#ifdef DEBUG2
        std::cerr << "Using " << num_points << " points\n";
        std::cerr << "screen " << s.width << " " << s.height << " " << s.fiducial_size << "\n";
#endif
    }

    double* error(double* fx, const std::vector<Point5D>::const_iterator b, const std::vector<Point5D>::const_iterator e, const BVL::Vector<double>& cop, const PlaneEquation& plane, const BVL::Matrix<double>& r, const BVL::Vector<double>& t) const
    {
        BVL::Vector<double> dp(3);
        std::vector<Point5D>::const_iterator it = b;
        while (it != e) {
            // for each screen point (sx,sy) in pixels, transform it
            // to the screen coord with the intrinsic parameters
            double sy = pixelsize * (it->sy - vcenter) / aspect_ratio;
            double sx = pixelsize * (it->sx - ucenter) - skew * sy;
            BVL::Vector<double> s(BVL::Tuple<double>(3, sx, -sy, 0.));
            BVL::Vector<double> s_world = r * s + t;

            // intersecting the plane with the line (left,s)
            // cop={cx,cy,cz},
            // s={sx,sy,sz},
            // planeN={nx,ny,nz},
            // planeD=d
            // cop + (s - cop) t /.
            // Solve[(cop + (s - cop) t).planeN + planeD == 0, t]
            // gives
            //{
            //cx+((d+cx nx+cy ny+cz nz) (-cx+sx))/(cx nx+cy ny+cz nz-nx sx-ny sy-nz sz),
            //cy+((d+cx nx+cy ny+cz nz) (-cy+sy))/(cx nx+cy ny+cz nz-nx sx-ny sy-nz sz),
            //cz+((d+cx nx+cy ny+cz nz) (-cz+sz))/(cx nx+cy ny+cz nz-nx sx-ny sy-nz sz)}
            double cd = dot_prod(cop, plane.normal);
            double denom = cd - dot_prod(s_world, plane.normal);
            cd += plane.distance;
            dp(1) = cop(1) + cd * (s_world(1) - cop(1)) / denom;
            dp(2) = cop(2) + cd * (s_world(2) - cop(2)) / denom;
            dp(3) = cop(3) + cd * (s_world(3) - cop(3)) / denom;

            *fx++ = dp(1) - it->x;
            *fx++ = dp(2) - it->y;
            *fx++ = dp(3) - it->z;

            ++it;
        }

        return fx;
    }

    void operator()(double* fx, double* x) const
    {
        double* fx_pointer = fx;
        // x components:
        // 1-3 left eye position in the screen coord
        // 4-6 right eye position in the screen coord
        // 7-9 euler angles of the rotation from screen to the world
        // 10-12 translation from screen to the world coord
        // 13 aspect ratio of the screen pixels
        // 14 skew of the screen pixels
        // 15 pixel size in u direction

        BVL::Vector<double> left(BVL::Tuple<double>(3, x[0], x[1], x[2]));
        BVL::Vector<double> right(BVL::Tuple<double>(3, x[3], x[4], x[5]));
        BVL::Matrix<double> r = BVL::euler2matrix(x[6], x[7], x[8]);
        BVL::Vector<double> t(3);
        t(1) = x[9];
        t(2) = x[10];
        t(3) = x[11];
        aspect_ratio = x[12];
        skew = x[13];
        pixelsize = x[14];
        BVL::Vector<double> left_world = r * left + t;
        BVL::Vector<double> right_world = r * right + t;

        fx_pointer = error(fx_pointer, left_top_samples.begin(), left_top_samples.end(), left_world, top_plane, r, t);
        fx_pointer = error(fx_pointer, left_bottom_samples.begin(), left_bottom_samples.end(), left_world, bottom_plane, r, t);
        fx_pointer = error(fx_pointer, right_top_samples.begin(), right_top_samples.end(), right_world, top_plane, r, t);
        fx_pointer = error(fx_pointer, right_bottom_samples.begin(), right_bottom_samples.end(), right_world, bottom_plane, r, t);

    }

private:
    const std::vector<Point5D> &left_top_samples, &left_bottom_samples;
    const std::vector<Point5D> &right_top_samples, &right_bottom_samples;
    PlaneEquation top_plane, bottom_plane;
    double ucenter, vcenter;
    mutable double aspect_ratio, skew, pixelsize;
    int num_points;
    int counter;
};

#endif /*_OBJECTLEASTSQUARES_H_*/


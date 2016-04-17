#ifndef _ROBUST_H_
#define _ROBUST_H_

/**
 *@file Robust.h
 *@brief Minimizing a robust version of the object function.  Copied from Newton.h
 */
#include <iostream>
#include <vector>
#include <BVL/math/linalg/linalg.h>
#include <BVL/camera/PerspectiveCamera.h>
#include "Point5D.h"
#include "PlaneFit.h"

extern BVL::Matrix<double> inverse2x2(const BVL::Matrix<double> &);

class OptFunc {
    public:
	OptFunc(const std::vector<Point5D> &lt, const std::vector<Point5D> &lb, const std::vector<Point5D> &rt, const std::vector<Point5D> &rb, const PlaneEquation &p1, const PlaneEquation &p2, const ScreenInfo &s)
	    :left_top_samples(lt),left_bottom_samples(lb),
	right_top_samples(rt),right_bottom_samples(rb),
	top_plane(p1),bottom_plane(p2),t(3,0.),dp(3,0.)
	{
	    ucenter = (s.width-1)/2.;
	    vcenter = (s.height-1)/2.;
	    counter = 0;
	    num_points = left_top_samples.size()+left_bottom_samples.size()+right_top_samples.size()+right_bottom_samples.size();

            std::cerr << "Using " << num_points << " points\n";

	}

	double error(const std::vector<Point5D>::const_iterator b, const std::vector<Point5D>::const_iterator e, const BVL::Vector<double> &cop, const PlaneEquation &plane)
	{
	    std::vector<Point5D>::const_iterator it=b;
	    double err = 0.;
	    while(it!=e) {
		// for each screen point (sx,sy) in pixels, transform it
		// to the screen coord with the intrinsic parameters
		double sy=pixelsize * (it->sy-vcenter)*aspect_ratio;
		double sx=pixelsize * (it->sx-ucenter)-skew*sy;
		BVL::Vector<double> s(BVL::Tuple<double>(3, sx,-sy,0.));
		// intersecting p1 (the top plane) with the line (left,s)
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
		double cd=dot_prod(cop,plane.normal);
		double denom=cd - dot_prod(s,plane.normal);
		cd += plane.distance;
                cd /=denom;
		dp(1)=cop(1)+cd*(s(1)-cop(1));
		dp(2)=cop(2)+cd*(s(2)-cop(2));
		dp(3)=cop(3)+cd*(s(3)-cop(3));
		dp = r*dp+t;

		dp(1) -= it->x;
		dp(2) -= it->y;
		dp(3) -= it->z;

                //double err1 = (it->dfx)*(it->dfx)+2*(it->dfx)*(it->dfy)+(it->dfy)*(it->dfy);
                double err2 = dp(1)*dp(1)+dp(2)*dp(2)+dp(3)*dp(3);

                //double n2 = err1+err2;
                double n2 = err2;
		err += log((1-exp(-n2/2))/n2);

		++it;
	    }

	    return -err;
	}

        void operator() (double *fx, double *x) {
            BVL::Vector<double> sol(15, x);
            *fx = f_to_minimize(sol);
        }

        double operator() (double *x) {
            BVL::Vector<double> sol(15, x);
            return f_to_minimize(sol);
        }

        double operator() (BVL::Vector<double> &x) {
            return f_to_minimize(x);
        }

	//double f_to_minimize(double *x)
	double f_to_minimize(BVL::Vector<double> &v) 
	{
	    const double *x = v.get_data();
	    // x components:
	    // 1-3 left eye position in the screen coord
	    // 4-6 right eye position in the screen coord
	    // 7-9 euler angles of the rotation from screen to the world
	    // 10-12 translation from screen to the world coord
	    // 13 aspect ratio of the screen pixels
	    // 14 skew of the screen pixels
	    // 15 pixel size in u direction

	    BVL::Vector<double> left(BVL::Tuple<double>(3, x[0],x[1],x[2]));
	    BVL::Vector<double> right(BVL::Tuple<double>(3, x[3],x[4],x[5]));
	    r=BVL::euler2matrix(x[6],x[7],x[8]);
	    t(1) = x[9]; t(2) = x[10]; t(3) = x[11];
	    aspect_ratio = x[12];
	    skew = x[13];
	    pixelsize = x[14];

	    // transform the planes to the screen coord with the new r and t
	    PlaneEquation p1,p2;
	    p1.normal = transpose(r)*top_plane.normal;
	    p1.distance = dot_prod(t,top_plane.normal)+top_plane.distance;
	    p2.normal = transpose(r)*bottom_plane.normal;
	    p2.distance = dot_prod(t,bottom_plane.normal)+bottom_plane.distance;

	    double total_error = 0.;
	    total_error+=error(left_top_samples.begin(),left_top_samples.end(),left,p1);
	    total_error+=error(left_bottom_samples.begin(),left_bottom_samples.end(),left,p2);
	    total_error+=error(right_top_samples.begin(),right_top_samples.end(),right,p1);
	    total_error+=error(right_bottom_samples.begin(),right_bottom_samples.end(),right,p2);

            /*
             * XXX, if use the robust version, it's -, otherwise, it's +
             */
            double iod = norm(left-right);
            //total_error -= (iod-63)*(iod-63)/(2*2.3*2.3);
            total_error += .1*(iod-63)*(iod-63)/(2*2.3*2.3);
           
	    return total_error/num_points;
	}


    private:
	const std::vector<Point5D> &left_top_samples,&left_bottom_samples;
	const std::vector<Point5D> &right_top_samples,&right_bottom_samples;
	PlaneEquation top_plane,bottom_plane;
	BVL::Matrix<double> r;
	BVL::Vector<double> t;
	BVL::Vector<double> dp; // difference between predicted 3D points and measured
        BVL::Matrix<double> icov_scr;
	double ucenter,vcenter;
	double aspect_ratio,skew,pixelsize;
	int num_points;
	int counter;
};

#endif/*_ROBUST_H_*/


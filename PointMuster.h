#ifndef _POINTMUSTER_H_
#define _POINTMUSTER_H_

/**
 *@file PointMuster.h
 *@brief 
 */
#include "Point5D.h"
#include <vector>
#include <assert.h>

class PointMuster
{
    public:
	PointMuster(const std::vector<double> &sx, const std::vector<double> &sy) { 
	    current_point = 0; 
	    assert(sx.size() == sy.size());
            x2d=sx;
            y2d=sy;
	}

	void get_screen_point(double &sx, double &sy) {
            assert(points.size() < x2d.size());
            current_point = points.size();
	    sx = x2d[current_point];
	    sy = y2d[current_point];
	}

	/**
	 * This call has to be matched with get_screen_point.
         * and current_point plays the (sole) role to sync
         * these two calls. In fact, it's not necessary, just using
         * points.size() would just be fine.
	 */
	void set_3d_correspondent(double x, double y, double z) {
            Point5D p;
            p.sx = x2d[current_point];
            p.sy = y2d[current_point];
            p.x = x;
            p.y = y;
            p.z = z;
            points.push_back(p);//effectively increased current_point
	}

	bool finished() const {
	    return points.size() == x2d.size();
	}

	const std::vector<Point5D> &get_points() const {
	    return points;
	}

        bool has_3d_correspondences() const {

            return !points.empty();
        }

        void pop_last_3d_correspondence() {
            assert(!points.empty());
            points.pop_back();
        }

        size_t size() {
            return points.size();
        }

    private:
	std::vector<Point5D> points;
	std::vector<double> x2d,y2d;
	int current_point;
};
	
#endif/*_POINTMUSTER_H_*/


#ifndef _PLANEFIT_H_
#define _PLANEFIT_H_

/**
 *@file PlaneFit.h
 *@brief 
 */
#include <BVL/math/linalg/linalg.h>
#include <vector>

struct PlaneEquation
{
    PlaneEquation() : normal(3,0.),distance(0) {}
    BVL::Vector<double> normal;
    double distance;
};

std::ostream &operator<<(std::ostream &os, const PlaneEquation &p);

class PlaneFit
{
    public:
	PlaneFit();
	~PlaneFit() {}

	void add_point(double x, double y, double z);
	void add_point(const BVL::Vector<double> &x);

	// The following function are not const because they all
	// call fit_plane, which is not const.
	PlaneEquation get_plane_equation();
	const std::vector<double> &get_residuals();
	double get_mean_sum_of_squared_errors();

    private:
	void fit_plane();

	std::vector<BVL::Vector<double> > points;
	PlaneEquation equation;
	std::vector<double> residuals;
	double mean_sse;
	bool new_point_added;
};
	
	
#endif/*_PLANEFIT_H_*/


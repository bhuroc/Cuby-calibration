/**
 *@file PlaneFit.cc
 *@brief 
 */
#include <iostream>
#include <numeric>
#include "PlaneFit.h"

using namespace BVL;
using namespace std;

std::ostream &operator<<(std::ostream &os, const PlaneEquation &p)
{
    os << p.normal << " " << p.distance;
    return os;
}

PlaneFit::PlaneFit()
{
    new_point_added = false;
}

void PlaneFit::add_point(double x, double y, double z)
{
    add_point(Vector<double>(Tuple<double>(3, x, y, z)));
}

void PlaneFit::add_point(const Vector<double> &v)
{
    new_point_added = true;
    points.push_back(v);
}

PlaneEquation PlaneFit::get_plane_equation() 
{
    if(new_point_added) {
	fit_plane();
    }

    return equation;
}

const std::vector<double> &PlaneFit::get_residuals() 
{
    if(new_point_added) {
	fit_plane();
    }

    return residuals;
}

double PlaneFit::get_mean_sum_of_squared_errors() 
{
    if(new_point_added) {
	fit_plane();
    }

    return mean_sse;
}

void PlaneFit::fit_plane() 
{
    new_point_added = false;

    int n = points.size();

    // move the centroid to origin
    Vector<double> centroid = accumulate(points.begin(), points.end(), Vector<double>(3, 0.));
    centroid /= static_cast<double>(n);
    //cerr << "centroid: " << centroid <<endl;

    Matrix<double> A(n, 3);
    vector<Vector<double> >::const_iterator i=points.begin();
    int r = 1;
    for(; i<points.end(); ++i) {
	Vector<double> x = (*i)-centroid;
	A(r, 1) = x(1);
	A(r, 2) = x(2);
	A(r, 3) = x(3);
	++r;
    }
    SVD<double> svd(A);
    Matrix<double> v = svd.get_v();

    equation.normal(1) = v(1,3);
    equation.normal(2) = v(2,3);
    equation.normal(3) = v(3,3);
    equation.distance = -(dot_prod(equation.normal,centroid));

    double smallest_eig_value = svd.get_singular_values()(3);
    smallest_eig_value *= smallest_eig_value;
    
    mean_sse = smallest_eig_value/static_cast<double>(n);

    // residual of each point
    residuals.clear();
    for(int k=0; k<n; ++k) 
	residuals.push_back(dot_prod(equation.normal, points[k])+equation.distance);

}


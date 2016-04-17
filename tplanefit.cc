/**
 *@file tplanefit.cc
 *@brief 
 */
#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <BVL/math/linalg/linalg.h>
#include <BVL/math/stats/rng.h>
#include "PlaneFit.h"

using namespace std;
using namespace BVL;

vector<Vector<double> > generate_plane_points(const Vector<double> &equation, const Vector<double> &noise, int num_points)
{
    vector<Vector<double> > points;
    
    double xmin = -100., xmax = 100.;
    double ymin = -100., ymax = 100.;
    double zmin = -100., zmax = 100.;

    RandomNumberGenerator xgen,ygen,zgen;
    xgen.set_distribution("uniform", xmin, xmax);
    xgen.seed(1);
    ygen.set_distribution("uniform", ymin, ymax);
    ygen.seed(2);
    zgen.set_distribution("uniform", zmin, zmax);
    zgen.seed(3);
    
    cerr << "plane equation " << equation << "\n";
    
    for(int i=0; i<num_points; ++i) {
	Vector<double> p(3);
	if(equation(3) != 0.) {
	    p(1) = xgen.generate();
	    p(2) = ygen.generate();
	    p(3) = (-equation(4)-equation(1)*p(1)-equation(2)*p(2))/equation(3);
	}else if(equation(2) != 0.) {
	    p(1) = xgen.generate();
	    p(3) = zgen.generate();
	    p(2) = (-equation(4)-equation(1)*p(1)-equation(3)*p(3))/equation(2);
	}else if(equation(1) != 0.) {
	    p(2) = ygen.generate();
	    p(3) = zgen.generate();
	    p(1) = (-equation(4)-equation(2)*p(2)-equation(3)*p(3))/equation(1);
	}else
	    return points;

	points.push_back(p);

    }

    // add noise
    if(norm(noise) != 0.) {
	Matrix<double> c(3,3, 0.);
	c(1,1) = noise(1);
	c(2,2) = noise(2);
	c(3,3) = noise(3);
	MultiNormalGenerator n(Vector<double>(3, "0 0 0"), c);

	for(int i=0; i<num_points; ++i) {
	    points[i] = points[i] + n.generate();
	}
    }

    return points;
}


int main()
{
    Vector<double> plane(4, "1 2 3 4");
    vector<Vector<double> > points = generate_plane_points(plane, Vector<double>(3, "1 1 1"), 100);

    //copy(points.begin(), points.end(), ostream_iterator<Vector<double> >(cout, "\n"));
    PlaneFit pf;
    for(int i=0; i<points.size(); ++i)
	pf.add_point(points[i]);

    PlaneEquation fit = pf.get_plane_equation();
    cerr << fit << "\n";
    Vector<double> fv(4,0.);
    fv(1) = fit.normal(1);
    fv(2) = fit.normal(2);
    fv(3) = fit.normal(3);
    fv(4) = fit.distance;
    if(norm(fv - plane/sqrt(14.)) <0.052)
	cerr << "Passed\n";
    else {
	cerr << "Failed\n";
	cerr << "true value " << plane/sqrt(14.) << "\t fit " << fit << "\n";
    }
    //cerr << pf.get_mean_residual();
    vector<double> r=pf.get_residuals();
    for(int i=0; i<r.size(); ++i)
	cerr << r[i] << " ";
    cerr << "\n";
}


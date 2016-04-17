#ifndef _POINT5D_H_
#define _POINT5D_H_

/**
 *@file Point5D.h
 *@brief 
 */
#include <iostream>

struct Point5D {
    Point5D() 
	:sx(0),sy(0),x(0),y(0),z(0),dfx(0),dfy(0) {}
    Point5D(double sx, double sy, double x, double y, double z)
	:sx(sx),sy(sy),x(x),y(y),z(z),dfx(0),dfy(0) {}
    double sx, sy;
    double x, y, z;
    double dfx, dfy; // difference from the fiducial values
};

std::ostream &operator<<(std::ostream &os, const Point5D &p);
std::istream &operator>>(std::istream &is, Point5D &p);

#endif/*_POINT5D_H_*/


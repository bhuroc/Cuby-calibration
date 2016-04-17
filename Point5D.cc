/**
 *@file Point5D.cc
 *@brief 
 */
#include "Point5D.h"

std::ostream &operator<<(std::ostream &os, const Point5D &p)
{
    os << p.sx << " " << p.sy << " " << p.x << " " << p.y << " " << p.z;
    return os;
}

std::istream &operator>>(std::istream &is, Point5D &p)
{
    is >> p.sx >> p.sy >> p.x >> p.y >> p.z;
    // usually should check if the input is finished all right
    // if (is) ...
    return is;
}



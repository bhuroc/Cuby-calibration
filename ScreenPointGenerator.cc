/**
 *@file ScreenPointGenerator.cc
 *@brief 
 */
#include <iostream>
#include <stdexcept>
#include <math.h>
#include <stdlib.h>
#include "ScreenPointGenerator.h"

using std::vector;
using std::cerr;
using std::endl;

ScreenPointGenerator::ScreenPointGenerator(int w, int h, int np, int mar, bool exact)
    :win_width(w), win_height(h), num_points(np), margin(mar)
{
    jitter_range = 30; // should be less than margin

    // generate num_calib_points uniformly distributed points
    // n is the number of point per row (along width)
    double n = ceil(sqrt(static_cast<double>(num_points) * win_width / win_height));
    double m = ceil(static_cast<double>(num_points) / n);

    cerr << "n: " << n << " m: " << m << "\n";
    //cerr << "margin: " << margin << "\n";

    // i < m instead of i <= m, as in j<=n, because I don't want to
    // push the points to the very bottom of the (virtual) screen.  It's
    // hard to see the points there.  The idea is to put the points all
    // over the screen to get a better fit.  Seems to me that covering
    // the rightmost part is more important than the bottom.
    //
    num_rows = m+1;
    num_cols = n+1;

    create_points(exact);
}

ScreenPointGenerator::ScreenPointGenerator(int w, int h, int n_rows, int n_cols, int mar[])
    :win_width(w), win_height(h), num_rows(n_rows), num_cols(n_cols)
{
    if(num_rows < 3 || num_cols <3) {
        cerr << "Too few points.\n";
        throw std::logic_error("Too few points.");
    }
    create_points(mar[0],mar[1],mar[2],mar[3]);
}

void ScreenPointGenerator::create_points(bool exact)
{

    // Don't add jitter on x to the rightmost column, or else the points
    // are pushed too far to the right and people are having problem
    // to see them.
    int x, y;
    for(int i=0; i<num_rows; ++i) {
	for(int j=0; j<=num_cols; ++j) {
	    int jitter = rand() % (2*jitter_range) - jitter_range;

            if(exact && screen_x.size() == (size_t)num_points)
                goto done;

	    if(j==num_cols)
		jitter = 0;
	    x = (ceil((win_width-2*margin)/double(num_cols)*j) + margin+jitter);
	    screen_x.push_back(x);

	    jitter = rand() % (2*jitter_range) - jitter_range;
	    y = (ceil((win_height-2*margin)/double(num_rows) * i) + margin+jitter);
	    screen_y.push_back(y);
	}
    }
    // recalculate num_calib_points
    num_points = screen_x.size();
    cerr << "final num of points " << num_points << "\n";

done:

#ifdef DEBUG
    vector<double>::iterator yi=screen_y.begin();
    vector<double>::iterator xi=screen_x.begin();
    vector<double>::iterator xend=screen_x.end();
    while(xi != xend) {
	cerr << *xi++ << " " << *yi++ << "\n";
    }
#endif
    return;
}

void ScreenPointGenerator::create_points(int left_margin, int right_margin, int top_margin, int bottom_margin)
{

    int x, y;
    int x_step = ceil((win_width-left_margin-right_margin)/double(num_cols-1));
    int y_step = ceil((win_height-top_margin-bottom_margin)/double(num_rows-1));
    for(int i=0; i<num_rows; ++i) {
	for(int j=0; j<num_cols; ++j) {

	    x = x_step*j + left_margin;
	    screen_x.push_back(x);

	    y = y_step * i + top_margin;
	    screen_y.push_back(y);
	}
    }

    num_points = screen_x.size();

#ifdef DEBUG
    vector<double>::iterator yi=screen_y.begin();
    vector<double>::iterator xi=screen_x.begin();
    vector<double>::iterator xend=screen_x.end();
    while(xi != xend) {
	cerr << *xi++ << " " << *yi++ << "\n";
    }
#endif
    return;
}

void ScreenPointGenerator::get_point(size_t i, double &x, double &y)
{
	if(i >= screen_x.size()){
		cerr << "Screen point index out of range.\n";
		throw std::out_of_range("Index out of range.");
	}else {
	x = screen_x[i];
	y = screen_y[i];
    }
}
    


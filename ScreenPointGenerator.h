#ifndef _SCREENPOINTGENERATOR_H_
#define _SCREENPOINTGENERATOR_H_

/**
 *@file ScreenPointGenerator.h
 *@brief 
 */
#include <vector>

class ScreenPointGenerator {
    public:
	ScreenPointGenerator(int w, int h, int n, int m=50, bool exact=false);
	ScreenPointGenerator(int w, int h, int n_rows, int n_cols, int m[]);

	void get_point(size_t i, double &x, double &y);
	int get_num_points() const { return num_points; }
	const std::vector<double> &get_x_points() const {
	    return screen_x;
	}

	const std::vector<double> &get_y_points() const {
	    return screen_y;
	}

	void set_margin(int m) {
	    margin = m;
	    jitter_range = m/3;
	}

    private:
	void create_points(bool exact);
	void create_points(int left, int right, int top, int bot);
	int win_width, win_height;
	int num_points;
	int num_rows, num_cols;
	int margin;
	int jitter_range;
	std::vector<double> screen_x, screen_y;
};

#endif/*_SCREENPOINTGENERATOR_H_*/


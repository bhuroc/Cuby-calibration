/**
 *@file filecalib.cc
 *@brief 
 */
#include <iostream>
#include <VML/System/IniParser.h>
#include "Calibrator.h"
#include "EM.h"
#include "RobustCalibrator.h"

using namespace std;
using namespace BVL;
using namespace VML;

int main(int argc, char **argv)
{
    IniParser *data_file;

    if(argc == 2) {
	data_file = new IniParser(argv[1]);
    }else {
	data_file = new IniParser(cin);
    }

    data_file->set_keyvalue_separator(':');
    if(data_file->read()) {
	cerr << "Not a valid file.\n";
	return 1;
    }

    int win_width = data_file->parse_int(data_file->get_value("Screen", "width"));
    int win_height = data_file->parse_int(data_file->get_value("Screen", "height"));
    int fiducial_size = data_file->parse_int(data_file->get_value("Screen", "fiducial size"));

    vector<Point5D> left_top,left_bottom, right_top, right_bottom;
    Point5D p;
    vector<string> *data = data_file->get_values("raw data left top");
    for(size_t i=0; i<data->size(); ++i) {
	istringstream is(data->at(i));
	is >> p;
	left_top.push_back(p);
	cout << i+1 << ": " << p << "\n";
    }
    delete data;

    data = data_file->get_values("raw data left bottom");
    for(size_t i=0; i<data->size(); ++i) {
	istringstream is(data->at(i));
	is >> p;
	left_bottom.push_back(p);
	cout << i+1 << ": " << p << "\n";
    }
    delete data;

    data = data_file->get_values("raw data right top");
    for(size_t i=0; i<data->size(); ++i) {
	istringstream is(data->at(i));
	is >> p;
	right_top.push_back(p);
	cout << i+1 << ": " << p << "\n";
    }
    delete data;

    data = data_file->get_values("raw data right bottom");
    for(size_t i=0; i<data->size(); ++i) {
	istringstream is(data->at(i));
	is >> p;
	right_bottom.push_back(p);
	cout << i+1 << ": " << p << "\n";
    }
    delete data;

    Calibrator calib(ScreenInfo(win_width,win_height,fiducial_size), .05, 10.);
    EMCalibrator em;
    RobustCalibrator robust;
    calib.set_nonlinear_calibrator(&robust);
    calib.calibrate(left_top,left_bottom,right_top,right_bottom);
    calib.write_result(cout);
    //calib.test_by_reprojection();
    //calib.test_by_meet3d();

    delete data_file;

    return 0;
}


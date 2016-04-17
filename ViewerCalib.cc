/**
 *@file ViewerCalib.cc
 *@brief 
 */
#define _USE_MATH_DEFINES
#include <windows.h>
#include <errno.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <math.h>
#include <string>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#define STAT _stat
#include <deque>
#include <GL/gl.h>
#include <GL/glu.h>
#include <BVL/math/linalg/linalg.h>
#include <VML/GUI/console.h>
#include <VML/Optotrak/Optotrak.h>
#include <VML/Graphics/utils.h>
#include <VML/System/Date.h>
#include "ViewerCalib.h"
#include "CalibrationManager.h"
#include "State.h"
#include "EM.h"
#include "RobustCalibrator.h"
#include "EM.h"

using namespace std;
using namespace VML;
using namespace BVL;

CalibrationManager mgr;

char *inter_collect_sound[] = {
    "put on the block"
};

ViewerCalib::ViewerCalib()
{
    calibrator = 0;
    console_opened = false;
}

ViewerCalib::~ViewerCalib()
{
    if(mgr.is_doing_verification() && viewer)
	delete viewer;
    else if(calibrator) // If quit prematurely, calibrator will be 0
        delete calibrator;
    if(console_opened)
        close_console();
    delete_states();

    Optotrak::shutdown();
}

void ViewerCalib::calibrate(const vector<Point5D> &lt, const vector<Point5D> &lb, const vector<Point5D> &rt, const vector<Point5D> &rb)
{
    ScreenInfo s(win_width, win_height, mgr.get_point_radius());
    calibrator = new Calibrator(s, 0.5, mgr.get_tool_thickness());
    RobustCalibrator robust;
    calibrator->set_nonlinear_calibrator(&robust);
    calibrator->calibrate(lt, lb, rt, rb);
    viewer = calibrator->get_viewer();
    write_result();
    renderer->set_viewer(viewer);
    info_string = calibrator->get_calibration_info();
}

int ViewerCalib::init(int argc, char **argv)
{
    if(getenv("TERM")==NULL) {
	console_opened = true;
	open_console();
    }

    mgr.print_config_info();

    // init optotrak
    try {

	Optotrak::initialize();

    } catch (exception &e) {

        string msg = e.what();
        MessageBox(NULL, msg.c_str(), "Optotrak Exception", MB_OK);
	return 1;
    } 

    SoDB::init();

    string sub_name;
    if(argc == 2) {
        sub_name = argv[1];
    }else {
        cout << "Subject's name:";
        cin >> sub_name;
    }

    mgr.set_subject_name(sub_name);
    win = new GLWindow(GLWindow::default_settings|GLW_FULLSCREEN|GLW_STEREO);
    win->add_event_handler(this);
    win_width = win->get_width();
    win_height = win->get_height();
    BringWindowToTop(win->get_hwnd());

    renderer = new Render(win->get_width(),win->get_height());
    win->add_painter(renderer);
    renderer->set_smoothing(true);
    renderer->set_doing_stereo(true);
    renderer->init_gl();

    // Create the optocollector
    State::init(this);
    StateCollect::init();

    if(mgr.is_doing_verification()) {

	try {
	    viewer = new ViewingTransforms(mgr.get_verification_name());
            renderer->set_viewer(viewer);

            // Construct the info text
            Vector<double> l=viewer->get_left_eye_in_world();
            Vector<double> r=viewer->get_right_eye_in_world();
            ostringstream os;
            os << "interocular distance:" << norm(l-r) << "\n";
            os << "screen center in world:" << viewer->get_screen_center_in_world() << "\n";
            os << "table in screen:"<<viewer->get_tabletop_equation_in_screen() << "\n";

            info_string = os.str();

	    current_state = StateVerification::get_instance();

	}catch (...) {
	    cerr << "Can't open "<<mgr.get_verification_name() << " for verification.\n";
	    return 1;
	}

    }else {
        current_state = StateLeftBottom::get_instance();
    }

    return 0;
}

void ViewerCalib::delete_states()
{
    if(!mgr.is_doing_verification()) {
        StateLeftBottom::delete_instance();
        StateLeftTop::delete_instance();
        StateRightBottom::delete_instance();
        StateRightTop::delete_instance();
        StateInterCollect::delete_instance();
        StateCalibration::delete_instance();
        StateCollect::shutdown();
    }
    StateVerification::delete_instance();
    State::shutdown();
}

bool ViewerCalib::idle(int )
{
    SoDB::getSensorManager()->processTimerQueue();
    SoDB::getSensorManager()->processDelayQueue(TRUE);

    WindowEvent e;
    if(!event_queue.empty()) {
        e = event_queue.front();
        event_queue.pop_front();
    }
    e.time_stamp = timer.get_elapsed_millisec();

    current_state = current_state->transition(e);

    renderer->set_scene(current_state->get_scene());
    renderer->draw();

    return true;
}


// XXX How to prevent double-click
int ViewerCalib::handle_event(const WindowEvent &e)
{
    if(e.type == WE_KEY && e.key == 27) {
        return 0;
    }else if(e.type == WE_MOUSE_DOWN || e.type == WE_KEY) {
        event_queue.push_back(e);
        return 1;
    }
    return 0;
}

void ViewerCalib::write_result()
{
    // If a file with the same name already exit, rename it
    string existing_filename = mgr.get_calib_filename();

    if(mgr.use_old_format()) {
        existing_filename += ".ext";
    }

    if(mgr.backup_existing_file()) {
        struct STAT s;
        if(STAT(existing_filename.c_str(), &s) == -1 ) {
            if(errno != ENOENT) {
                cerr << "Can't stat " << existing_filename<<endl;
            }  //else: file doesn't exist, safe to write result to
        }else { // rename the existing file
            // get the creation time
            string suffix=ctime(&s.st_mtime);
            // get rid of spaces, :, and trailing stuff
            cerr << existing_filename << " was created at " << suffix << "\n";
            string::size_type idx;
            while((idx=suffix.find_first_of(" :\n") )!= string::npos) {
                suffix.erase(idx, 1);
            }
            cerr << "or " << suffix << "\n";
            string new_name = existing_filename;
            string::size_type n=new_name.find(".txt");
            if(n != string::npos) 
                new_name.insert(n,suffix);
            else
                new_name+=suffix;
            if(rename(existing_filename.c_str(), new_name.c_str())){
                cerr << "Can't rename " << existing_filename << " to " << new_name << "\n";
            }
        }
    }

    // free reign after the above s**t.
    ofstream os((mgr.get_calib_filename()).c_str());

    if(mgr.use_old_format()) {
        VML::translate(os, *viewer);
        // save an extra copy
        std::string extra = mgr.get_calib_filename()+".ext";
        ofstream eos(extra.c_str());
        calibrator->write_result(eos);
        write_extra_info(eos);
    }else {

        calibrator->write_result(os);
        write_extra_info(os);

    }
}

void ViewerCalib::write_extra_info(ostream &os) const
{
        //extra info at the calibration, not Calibrator's responsibility.
        os << "[Subject info]\n";
        os << "name:"<<mgr.get_subject_name() << "\n";
        Date d;
        os << "time:"<<d.to_string() << "\n";
}


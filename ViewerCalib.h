#ifndef _VIEWERCALIB_H_
#define _VIEWERCALIB_H_

/**
 *@file ViewerCalib.h
 *@brief 
 */
#include <deque>
#include <VML/GUI/VMLApp.h>
#include <VML/GUI/GLWindow.h>
#include <VML/GUI/WindowEvent.h>
#include <VML/System/ElapsedTimer.h>
#include <VML/Graphics/ViewingTransforms.h>
#include "Calibrator.h"
#include "CalibrationManager.h"
#include "Render.h"

char *inter_collect_sound[];

class State;

class ViewerCalib : public VML::EventHandler, public VML::VMLApp
{
    public:
	ViewerCalib();
	~ViewerCalib();

	bool idle(int);
	int init(int argc, char **argv);
	int handle_event(const VML::WindowEvent &e);
	void calibrate(const std::vector<Point5D> &left_top, const std::vector<Point5D> &left_bottom, const std::vector<Point5D> &right_top, const std::vector<Point5D> &right_bottom);

	const VML::ViewingTransforms *get_viewer() const {
            return viewer;
        }

        void delete_states();

        int get_win_width() const {
            return win_width;
        }

        int get_win_height() const {
            return win_height;
        }

        const std::string &get_calibration_info() {
            return info_string;
        }
        
        void write_extra_info(std::ostream &os) const;

    private:
	void write_result();

        VML::GLWindow *win;
        Render*renderer;
	Calibrator *calibrator;
	State *current_state;
	VML::ElapsedTimer timer;
	const VML::ViewingTransforms *viewer;
        std::deque<VML::WindowEvent> event_queue;
        int win_width, win_height;

	bool console_opened;
        std::string info_string;
};


#endif/*_VIEWERCALIB_H_*/


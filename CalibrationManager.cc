/**
 *@file CalibrationManager.cc
 *@brief 
 */
#include "CalibrationManager.h"
#include <VML/System/IniParser.h>
#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace VML;

void CalibrationManager::print_config_info()
{
    cout << "----------------------------------------------------\n";
    cout << "*** Please check the tool parameters are correct ***\n";
    cout << "[tool]\n";
    calib_tool->dump(cout);
    cout << "--- Other calibration parameters ---\n";
    cout << "[file]\n";
    output_file->dump(cout);
    cout << "[fiducial points]\n";
    calib_points->dump(cout);
    cout << "[screen]\n";
    screen->dump(cout);
    cout << "----------------------------------------------------\n";
}

CalibrationManager::CalibrationManager()
{
    output_file = new ConfigItem("file",
        "format", "new",
        "prefix", "calib_",
        "suffix", "txt",
        "backup", "true", 0);

    log_file = new ConfigItem("log",
        "filename", "log.txt",
        "enabled", "false", 0);

    calib_tool = new ConfigItem("tool",
        "type", "rigid",
        "name", "cuby1",
        "num markers", "3",
        // The tip of the cone to the bottom of the cuby is 9.65mm
        // The cuby itself is 1in, so the totaly is 9.65+25.4=35.05
        "thickness", "35.05", 0);

    calib_points = new ConfigItem("fiducial points",
        "num points", "3 4",
        "box size", "40", // the size of the blinking box
        "radius", "6", 0);

    screen = new ConfigItem("screen",
        "margins", "100 100 100 150", 0);

    // look for config file in current directory
    // the format is the header file
    IniParser config("calibrationcfg.txt");
    if (config.read() == 0) {

        //// get the file info
        output_file->read(config);

        //// get the tool
        calib_tool->read(config);

        //// number of calibration points
        calib_points->read(config);

        //// log or not
        log_file->read(config);

        //// margin that the screen points are away from the borders
        screen->read(config);

#ifdef DEBUG
        cerr << "== output file \n";
        output_file->dump(cerr);
        cerr << "== tool \n";
        calib_tool->dump(cerr);
#endif
    } else {
        cerr << "Can't find calibrationcfg.txt.  Use default values.\n";
    }

    verification = false;
}

void CalibrationManager::set_subject_name(const string& name)
{
    subject_name = name;
    if (subject_name.compare(0, 2, "v_") == 0) {
        verification = true;
        verification_name.assign(subject_name, 2, string::npos);
        cerr << "Verifying " << verification_name << "\n";
    } else {
        calib_name = output_file->get_value("prefix") + subject_name + "." + output_file->get_value("suffix");
    }
}


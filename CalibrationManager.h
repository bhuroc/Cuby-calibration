#ifndef _CALIBRATIONMANAGER_H_
#define _CALIBRATIONMANAGER_H_

/**
 *@file CalibrationManager.h
 *@brief
 */
#include <VML/System/ConfigItem.h>
#include <VML/System/IniParser.h>
#include <VML/System/StringTokenizer.h>
#include <map>

// look for config file in current directory
// [file]
// format=old/new
// prefix=
// suffix=
// [tool]
// name = cuby1
// num markers = 3
// thickness = 0
// [fiducial points]
// num points = 3 5
// radius = 6
// [screen]
// margins = 50 50 50 50
// [log]
// filename=calib.log

class CalibrationManager {
public:
    enum Tool {
        RIGID_BODY,
        SINGLE_MARKER
    };

    CalibrationManager();
    ~CalibrationManager()
    {
        delete calib_tool;
        delete log_file;
        delete screen;
        delete output_file;
        delete calib_points;
    }

    void print_config_info();

    void set_subject_name(const std::string& n);

    const std::string get_calib_filename() const
    {
        return calib_name;
    }

    const std::string get_subject_name() const
    {
        return subject_name;
    }

    bool is_using_rigid_body() const
    {
        return calib_tool->get_value("type") == "rigid";
    }

    const std::string get_tool_name() const
    {
        return calib_tool->get_value("name");
    }

    int get_num_markers() const
    {
        return atoi(calib_tool->get_value("num markers").c_str());
    }

    double get_tool_thickness() const
    {
        return atof(calib_tool->get_value("thickness").c_str());
    }

    bool is_log_enabled() const
    {
        return log_file->get_value("enabled") == "true";
    }

    bool is_doing_verification() const
    {
        return verification;
    }

    bool backup_existing_file() const
    {
        return output_file->get_value("backup") != "false";
    }

    const char* get_verification_name() const
    {
        return verification_name.c_str();
    }

    const std::string get_log_name() const
    {
        return log_file->get_value("filename");
    }

    void get_num_points(int& rows, int& cols)
    {
        VML::StringTokenizer<int> tokens(calib_points->get_value("num points"));
        rows = tokens.get_next_token();
        cols = tokens.get_next_token();
    }

    int get_point_radius() const
    {
        return atoi(calib_points->get_value("radius").c_str());
    }

    void get_screen_point_margins(int m[]) const
    {
        VML::StringTokenizer<int> tokens(screen->get_value("margins"));
        for (int i = 0; i < 4; ++i) {
            m[i] = tokens.get_next_token();
        }
    }

    bool use_old_format() const
    {
        return output_file->get_value("format") == "old";
    }

    // Size (pixels) of the blinking box
    int get_box_size() const
    {
        return s2i(calib_points->get_value("box size"));
    }

private:
    VML::ConfigItem *calib_tool, *log_file, *screen, *output_file, *calib_points;

    std::string subject_name;
    std::string calib_name;
    bool verification;
    std::string verification_name;
};

#endif /*_CALIBRATIONMANAGER_H_*/


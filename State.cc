/**
 *@file State.cc
 *@brief 
 */
#include "State.h"
#include "CalibrationManager.h"
#include "PointMuster.h"
#include "ViewerCalib.h"
#include <BVL/math/linalg/linalg.h>
#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/engines/SoSelectOne.h>
#include <Inventor/fields/SoMFInt32.h>
#include <Inventor/nodes/SoAsciiText.h>
#include <Inventor/nodes/SoBlinker.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoInfo.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTranslation.h>
#include <VML/Graphics/Cubies.h>
#include <VML/Graphics/inventor.h>
#include <VML/Optotrak/Optotrak.h>
#include <iostream>
#include <sstream>

using namespace VML;
using namespace std;

#include "axis.iv"
#include "reference_point.iv"

extern CalibrationManager mgr;
Cubies cubies;
ViewerCalib* State::vc = 0;
OptoCollector* State::collector = 0;

// The abstract states are not here
NEWSTATE_SOURCE(LeftBottom);
NEWSTATE_SOURCE(RightBottom);
NEWSTATE_SOURCE(LeftTop);
NEWSTATE_SOURCE(RightTop);
NEWSTATE_SOURCE(InterCollect);
NEWSTATE_SOURCE(Calibration);
NEWSTATE_SOURCE(Verification);
NEWSTATE_SOURCE(Test);

void State::init(ViewerCalib* v)
{
    vc = v;

    try {
        collector = new OptoCollector;
        if (mgr.is_using_rigid_body()) {
            collector->add_rigid_body(mgr.get_tool_name().c_str(), mgr.get_num_markers(), 0, VML::QUATERNION);
        } else {
            collector->add_markers(1);
        }
        collector->setup_collection();
        collector->activate();
    } catch (VML::OptotrakException& e) {
        throw;
    }
}

void State::shutdown()
{
    collector->deactivate();
    delete collector;
}

///////////////////////////////////////////////////////////////////
//
// The ancester of all collecting states
//
///////////////////////////////////////////////////////////////////
SoSeparator* StateCollect::reference = 0;
SoTranslation* StateCollect::point_position = 0;
ScreenPointGenerator* StateCollect::gen = 0;

void StateCollect::shutdown()
{
    delete gen;
}

void StateCollect::init()
{
    int n_rows, n_cols;
    int margins[4];
    mgr.get_num_points(n_rows, n_cols);
    mgr.get_screen_point_margins(margins);
    gen = new ScreenPointGenerator(vc->get_win_width(), vc->get_win_height(), n_rows, n_cols, margins);

    SoInput in;
    in.setBuffer(reference_point, strlen(reference_point));
    reference = SoDB::readAll(&in);

    // Find the node for positioning the points
    point_position = (SoTranslation*)SoNode::getByName("point_position");

    // Set the size of the screen
    SoScale* border_size = (SoScale*)SoNode::getByName("screen_size");
    border_size->scaleFactor.setValue(vc->get_win_width(), vc->get_win_height(), 1);

    // Set the size of the blinker
    SoScale* box_size = (SoScale*)SoNode::getByName("box_size");
    box_size->scaleFactor.setValue(mgr.get_box_size(), mgr.get_box_size(), 1);

    // Set the size of the point
    SoScale* p_size = (SoScale*)SoNode::getByName("point_size");
    p_size->scaleFactor.setValue(mgr.get_point_radius(), mgr.get_point_radius(), 1.);
}

StateCollect::~StateCollect()
{
    delete point_list;
}

int StateCollect::make_transition(const WindowEvent& e, State* from)
{
    if (e.type == WE_MOUSE_DOWN && e.button == WE_LEFT_BUTTON) {

        // get the 3D correspondence
        BVL::Vector<double> p(3);
        int fn = collector->update_frame();
        if (fn < 0 || collector->get_position(p.get_data(), 0) < 0) {
            Beep(700, 300);
            Beep(700, 300);
            return 0;
        } else {
            Beep(450, 100);
            point_list->set_3d_correspondent(p(1), p(2), p(3));
        }

        if (point_list->finished()) {
            return 1; // goto next state
        } else {
            double x, y;
            point_list->get_screen_point(x, y);
            point_position->translation.setValue(x, y, 0);
        }

    } else if ((e.type == WE_KEY && e.key == 'r') || (e.type == WE_MOUSE_DOWN && e.button == WE_RIGHT_BUTTON)) {

        // Rewind and it does rewinding beyond state boundary.  If
        // I rewind to the first point of right_bottom, I should
        // go into left_bottom state.  Add a from state.
        if (point_list->has_3d_correspondences()) {
            point_list->pop_last_3d_correspondence();
            double x, y;
            point_list->get_screen_point(x, y);
            point_position->translation.setValue(x, y, 0);
        } else if (from) {
            cerr << "Back to prevous state.\n";
            StateCollect* from_state = dynamic_cast<StateCollect*>(from);
            from_state->point_list->pop_last_3d_correspondence();
            return -1; // to back to previous state
        } else {
            cerr << "You're already at the very beginning.\n";
            return 0;
        }
    }

    return 0; // 0: stays at the same state
}

///////////////////////////////////////////////////////////////////
//
// Left bottom
//
///////////////////////////////////////////////////////////////////
StateLeftBottom::StateLeftBottom()
{
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, l, SoInfo, string, "left buffer");
    l->setName("left_buffer");
    root->addChild(StateCollect::reference);

    // my own point list
    point_list = new PointMuster(gen->get_x_points(), gen->get_y_points());

    wait_for_first_click = true;

    // inherit the scene graph
    double x, y;
    point_list->get_screen_point(x, y);
    point_position->translation.setValue(x, y, 0);
}

State* StateCollect::change_state(State* to)
{
    StateCollect* s = dynamic_cast<StateCollect*>(to);
    double x, y;
    s->point_list->get_screen_point(x, y);
    point_position->translation.setValue(x, y, 0);
    return to;
}

State* StateLeftBottom::transition(const WindowEvent& e)
{
    if (wait_for_first_click && e.type == WE_MOUSE_DOWN) {
        wait_for_first_click = false;
        // Find the DrawStyle node and disable the screen border
        SoDrawStyle* ds = (SoDrawStyle*)SoNode::getByName("screen_border");
        ds->style = SoDrawStyle::INVISIBLE;
    }

    switch (make_transition(e, 0)) {
    case 0:
        return this;
    case 1:
        // We need to explicitly update point_position.
        // In normal operation, point_position of the next
        // state is set by its constructor and it's good
        // but if the state is rewound to, point_position
        // still has the from-state's last point position.
        return change_state(StateRightBottom::get_instance());

    default:
        assert("LeftBottom: shouldn't be here" && 0);
        return 0;
    }
}

///////////////////////////////////////////////////////////////////
//
// Right bottom
//
///////////////////////////////////////////////////////////////////
StateRightBottom::StateRightBottom()
{
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, l, SoInfo, string, "right buffer");
    l->setName("right_buffer");
    root->addChild(StateCollect::reference);

    point_list = new PointMuster(gen->get_x_points(), gen->get_y_points());
    double x, y;
    point_list->get_screen_point(x, y);
    point_position->translation.setValue(x, y, 0);
}

State* StateRightBottom::transition(const WindowEvent& e)
{
    switch (make_transition(e, StateLeftBottom::get_instance())) {
    case 0:
        return this;
    case 1:
        return StateInterCollect::get_instance();
    case -1:
        return change_state(StateLeftBottom::get_instance());
    }
    assert("RightBottom runaway" && 0);
    return this;
}

///////////////////////////////////////////////////////////////////
//
// Left top
//
///////////////////////////////////////////////////////////////////
StateLeftTop::StateLeftTop()
{
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, l, SoInfo, string, "left buffer");
    l->setName("left_buffer");
    root->addChild(StateCollect::reference);

    point_list = new PointMuster(gen->get_x_points(), gen->get_y_points());
    double x, y;
    point_list->get_screen_point(x, y);
    point_position->translation.setValue(x, y, 0);
}

State* StateLeftTop::transition(const WindowEvent& e)
{
    switch (make_transition(e, StateRightBottom::get_instance())) {
    case 0:
        return this;
    case 1:
        return change_state(StateRightTop::get_instance());
    case -1:
        return change_state(StateRightBottom::get_instance());
    }
    assert("LeftTop runaway" && 0);
    return this;
}

///////////////////////////////////////////////////////////////////
//
// Right top
//
///////////////////////////////////////////////////////////////////
StateRightTop::StateRightTop()
{
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, l, SoInfo, string, "right buffer");
    l->setName("right_buffer");
    root->addChild(StateCollect::reference);

    point_list = new PointMuster(gen->get_x_points(), gen->get_y_points());
    double x, y;
    point_list->get_screen_point(x, y);
    point_position->translation.setValue(x, y, 0);
}

State* StateRightTop::transition(const WindowEvent& e)
{
    switch (make_transition(e, StateLeftTop::get_instance())) {
    case 0:
        return this;
    case 1:
        return StateCalibration::get_instance();
    case -1:
        return change_state(StateLeftTop::get_instance());
    }
    assert("RightTop runaway" && 0);
    return this;
}

///////////////////////////////////////////////////////////////////
//
// Inter-collect
//
///////////////////////////////////////////////////////////////////
StateInterCollect::StateInterCollect()
{
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, l, SoInfo, string, "left buffer");
    l->setName("left_buffer");
    ADD_NODE(root, lm, SoLightModel, model, SoLightModel::BASE_COLOR);
    ADD_NODE(root, red, SoMaterial, diffuseColor, SbVec3f(1, 0, 0));
    ADD_NODE2(root, f, SoFont, name, "Arial", size, 28);
    ADD_NODE(root, tr, SoTranslation, translation, SbVec3f(vc->get_win_width() / 2, vc->get_win_height() / 2, 0.));
    ADD_NODE2(root, r, SoRotationXYZ, axis, SoRotationXYZ::X, angle, M_PI);
    ADD_NODE(root, b, SoBlinker, speed, 1);
    ADD_NODE2(b, t, SoAsciiText, string, "Put On An Extra Block", justification, SoAsciiText::CENTER);
    Beep(261, 120);
    Beep(330, 120);
    Beep(392, 120);
}

State* StateInterCollect::transition(const WindowEvent& e)
{
    if (e.type == WE_MOUSE_DOWN) {
        return change_state(StateLeftTop::get_instance());
        //return StateLeftTop::get_instance();
    }
    return this;
}

///////////////////////////////////////////////////////////////////
//
// Calibration
//
///////////////////////////////////////////////////////////////////
StateCalibration::StateCalibration()
{
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, l, SoInfo, string, "right buffer");
    l->setName("right_buffer");
    ADD_NODE(root, lm, SoLightModel, model, SoLightModel::BASE_COLOR);
    ADD_NODE(root, red, SoMaterial, diffuseColor, SbVec3f(1, 0, 0));
    ADD_NODE2(root, f, SoFont, name, "Times New Roman", size, 24);
    ADD_NODE(root, tr, SoTranslation, translation, SbVec3f(vc->get_win_width() / 2, vc->get_win_height() / 2, 0.));
    ADD_NODE2(root, r, SoRotationXYZ, axis, SoRotationXYZ::X, angle, M_PI);
    ADD_NODE2(root, t, SoAsciiText, string, "Calculating ...", justification, SoAsciiText::CENTER);
}

State* StateCalibration::transition(const WindowEvent& e)
{
    // Note that get_points() returns a reference to the points,
    // so it's important that the states are around when
    // calibration is under way.
    vc->calibrate(
        ((StateCollect*)(StateLeftTop::get_instance()))->get_points(),
        ((StateCollect*)(StateLeftBottom::get_instance()))->get_points(),
        ((StateCollect*)(StateRightTop::get_instance()))->get_points(),
        ((StateCollect*)(StateRightBottom::get_instance()))->get_points());

    return StateVerification::get_instance();
}

///////////////////////////////////////////////////////////////////
//
// Verification
//
///////////////////////////////////////////////////////////////////
StateVerification::~StateVerification()
{
    if (recorder)
        delete recorder;
    //delete filter
}

StateVerification::StateVerification()
{
    recorder = 0;
    recording = false;
    //filter = new Kalman6DPredictor(collector);
    // XXX initialize the filter, set up the model and
    // prediction steps.
    //collector->set_filter(filter);

    // NOTE: this constructor is called either after the calibration
    // is done or we're doing a verification.  This is guarantteed
    // by the delayed construction (get_instance() won't be called
    // until viewer is valid).
    const ViewingTransforms* viewer = vc->get_viewer();
    root = new SoSeparator;
    root->ref();

    ADD_NODE(root, lm, SoLightModel, model, SoLightModel::BASE_COLOR);
    ADD_NODE(root, red, SoMaterial, diffuseColor, SbVec3f(1, 0, 0));

    SoInput in;
    in.setBuffer(axis_iv, strlen(axis_iv));
    SoSeparator* axes = SoDB::readAll(&in);

    SoSeparator* cuby;
    if (mgr.is_using_rigid_body()) {
        const char* civ = cubies.get_iv_description(mgr.get_tool_name().c_str());
        if (civ) {
            in.setBuffer(const_cast<char*>(civ), strlen(civ));
            cuby = SoDB::readAll(&in);
        } else {
            cerr << "Tool name's wrong (" << mgr.get_tool_name() << ").  How do you get this far?\n";
        }
    }

    on_off = new SoOnOff;
    SoSelectOne* selectone = new SoSelectOne(SoMFInt32::getClassTypeId());
    ((SoMFInt32*)selectone->input)->set1Value(0, SO_SWITCH_NONE);
    ((SoMFInt32*)selectone->input)->set1Value(1, SO_SWITCH_ALL);
    selectone->index.connectFrom(&on_off->isOn);

    ADD_NODE0(root, show_axes, SoSwitch);
    ADD_NODE2(show_axes, f1, SoFont, name, "Arial", size, 18);
    show_axes->whichChild.connectFrom(selectone->output);

    // Table axes
    ADD_NODE0(show_axes, tab, SoSeparator);
    ADD_NODE0(tab, tab2scr, SoMatrixTransform);
    tab2scr->matrix = fill_matrix(viewer->get_tabletop_to_screen_matrix());
    tab->addChild(axes);
    ADD_NODE(tab, raise_text, SoTranslation, translation, SbVec3f(0, -10, 0.));
    ADD_NODE(tab, tab_text, SoAsciiText, string, "Table");

    // Screen axes
    ADD_NODE0(show_axes, scr, SoSeparator);
    // shrink the screen axes, so I can differentiate to the table ones
    // And text too.
    ADD_NODE(scr, shrink, SoScale, scaleFactor, SbVec3f(0.5, 0.5, 0.5));
    ADD_NODE(scr, scr_text, SoAsciiText, string, "Screen");
    scr->addChild(axes);

    // Some information
    ADD_NODE0(show_axes, info, SoSeparator);
    ADD_NODE2(info, f, SoFont, name, "Courier New", size, 10);
    ADD_NODE(info, text_pos, SoTranslation, translation, SbVec3f(0, -60, 0));
    ADD_NODE0(info, info_text, SoAsciiText);
    info_text->justification = SoAsciiText::CENTER;
    istringstream is(vc->get_calibration_info());
    int nlines = 0;
    while (is) {
        string aline;
        getline(is, aline);
        SbString s = aline.c_str();
        info_text->string.set1Value(nlines++, s);
    }

    rb2opto_engine = new SoComposeMatrix;

    ADD_NODE0(root, cub, SoSeparator);
    ADD_NODE0(cub, opto2scr, SoMatrixTransform);
    opto2scr->matrix = fill_matrix(viewer->get_world_to_screen_matrix());
    ADD_NODE0(cub, rb2opto, SoMatrixTransform);
    rb2opto->matrix.connectFrom(&(rb2opto_engine->matrix));

    // the tip
    ADD_NODE0(cub, d, SoSeparator);
    ADD_NODE(d, dot, SoSphere, radius, 1);

    // cuby
    if (mgr.is_using_rigid_body()) {
        ADD_NODE0(cub, show_cuby, SoSwitch);
        //show_cuby->whichChild.connectFrom(selectone->output);
        show_cuby->whichChild = 0;
        show_cuby->addChild(cuby);
    }
}

State* StateVerification::transition(const WindowEvent& e)
{
    int frame_number = collector->update_frame();
    BVL::Vector<double> x(3, 0.), v(3, 0.);
    BVL::Vector<double> predicted_x(3, 0.), predicted_q(4, 0.);
    BVL::Vector<double> q(4, 0.);
    if (mgr.is_using_rigid_body()) {
        BVL::Pose d;
        collector->get_pose(d, 0);
        x = d.get_translation();
        q = d.get_quaternion();

    } else {
        collector->get_position(x.get_data());
    }

    rb2opto_engine->translation = SbVec3f(x[0], x[1], x[2]);

    if (mgr.is_using_rigid_body()) {
        rb2opto_engine->rotation.setValue(q(2), q(3), q(4), q(1));
    }

    if (recording) {
        *recorder << frame_number << " " << x << " " << q << "\n";
    }

    // Toggle the full view and recording
    if (e.type == WE_MOUSE_DOWN) {
        if (e.button == WE_LEFT_BUTTON) {
            on_off->toggle.setValue();
        } else if (e.button == WE_RIGHT_BUTTON) {
            recording = !recording;
            if (recording && recorder == 0) {
                recorder = new ofstream("trackingdata.txt");
            }
        }
    }
    return this;
}

StateTest::StateTest()
{
}

State* StateTest::transition(const WindowEvent& e)
{
    return this;
}

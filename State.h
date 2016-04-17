#ifndef _STATE_H_
#define _STATE_H_

#include <vector>
#include <BVL/math/linalg/linalg.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoBlinker.h>
#include <Inventor/engines/SoOnOff.h>
#include <Inventor/engines/SoComposeMatrix.h>
#include <VML/Optotrak/OptoCollector.h>
#include <VML/GUI/WindowEvent.h>
#include "ScreenPointGenerator.h"
#include "PointMuster.h"

class ViewerCalib;

/**
 *@file State.h
 *@brief 
 */
class State
{
    protected:
        State() {}
    public:
        virtual ~State() { root->unref();} 
        virtual State *transition(const VML::WindowEvent &e) = 0;
        SoSeparator *get_scene() {
            return root; 
        }
        static void init(ViewerCalib *);
        static void shutdown();

    protected:
        SoSeparator *root;
        static ViewerCalib *vc;
        static VML::OptoCollector *collector;
};



#define NEW_CONCRETE_STATE(state) \
    protected: \
        State##state();\
    public:\
        static State *get_instance() {\
            if(instance == 0) \
               instance = new State##state;\
            return instance;\
        }\
        static void delete_instance() {\
            if(instance) \
                delete instance;\
        }\
        State *transition(const VML::WindowEvent &e);\
    private:\
        static State *instance


#define NEWSTATE_SOURCE(state) State *State##state::instance=0


class StateCollect : public State
{
    protected:
        StateCollect() {
            point_list = 0;
            root = 0;
        }
    public:
        static void init();
        static void shutdown();

        State *change_state(State *);
        ~StateCollect();
        const std::vector<Point5D> &get_points() const {
            return point_list->get_points();
        }

    protected:
        int make_transition(const VML::WindowEvent &e, State *from);
        static ScreenPointGenerator *gen;
        PointMuster *point_list;
        static SoSeparator *reference;
        static SoTranslation *point_position;
};

class StateCalibration: public State
{
    NEW_CONCRETE_STATE(Calibration);
};

class StateLeftBottom : public StateCollect
{
    NEW_CONCRETE_STATE(LeftBottom);
    private:
        bool wait_for_first_click;
};

class StateRightBottom : public StateCollect
{
    NEW_CONCRETE_STATE(RightBottom);
};

class StateLeftTop: public StateCollect
{
    NEW_CONCRETE_STATE(LeftTop);
};

class StateRightTop: public StateCollect
{
    NEW_CONCRETE_STATE(RightTop);
};

class StateVerification: public State
{
    NEW_CONCRETE_STATE(Verification);
    ~StateVerification();
    private:
        SoOnOff *on_off;
        SoComposeMatrix *rb2opto_engine;
        bool recording;
        //KinematicPredictor *filter;
        std::ostream *recorder;
};

class StateInterCollect: public StateCollect
{
    NEW_CONCRETE_STATE(InterCollect);
};

class StateTest: public State
{
    NEW_CONCRETE_STATE(Test);
    private:
    SoBlinker *b;
};


#endif/*_STATE_H_*/


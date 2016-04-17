/**
 *@file Render.cc
 *@brief 
 */
#include <iostream>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoNode.h>
#include "Render.h"

using namespace std;

void Render::init_gl()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    SbVec2s w = viewport->getWindowSize();
    glOrtho(0, w[0], 0, w[1], -1.f, 1.f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Render::draw()
{
    if(viewer) 
        RenderManager::draw();
    else {

        glDrawBuffer(GL_BACK_RIGHT);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
        glDrawBuffer(GL_BACK_LEFT);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

        // search for buffer directives
        if(scene) {
            SoNode *info = ((SoSeparator*)scene)->getChild(0);
            if(info->getName() == "left_buffer") {
                glDrawBuffer(GL_BACK_LEFT);
                render->apply(scene);
            }
            if(info->getName() == "right_buffer") {
                glDrawBuffer(GL_BACK_RIGHT);
                render->apply(scene);
            }
        }

        swap_buffers();
    }
}


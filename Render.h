#ifndef _RENDER_H_
#define _RENDER_H_

/**
 *@file Render.h
 *@brief 
 */
#include <VML/GUI/RenderManager.h>

class Render : public VML::RenderManager
{
    public:
        Render(int w, int h) : RenderManager(w, h) {}
        void init_gl();
        void draw();
};

#endif/*_RENDER_H_*/


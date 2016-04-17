/**
 *@file main.cpp
 *@brief 
 */
#include <iostream>
#include "ViewerCalib.h"

int main(int argc, char **argv)
{
    ViewerCalib vc;

    if(vc.init(argc, argv)) {
        std::cerr << "Can't init the application.\n";
        return 1;
    }

    return vc.main_loop();
}


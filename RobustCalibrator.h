#ifndef _ROBUSTCALIBRATOR_H_
#define _ROBUSTCALIBRATOR_H_

/**
 *@file RobustCalibrator.h
 *@brief 
 */
#include <iostream>
#include <BVL/math/linalg/linalg.h>
#include "NonlinearCalibrator.h"
#include "Calibrator.h"

class RobustCalibrator: public NonlinearCalibrator
{
    public:
        void calibrate(Calibrator *c);
        void write_info(std::ostream &os);
    private:
        Calibrator *calibrator;
        BVL::Vector<double> gradient;
};

#endif/*_ROBUSTCALIBRATOR_H_*/


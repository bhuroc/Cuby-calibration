#ifndef _NONLINEARCALIBRATOR_H_
#define _NONLINEARCALIBRATOR_H_

/**
 *@file NonlinearCalibrator.h
 *@brief 
 */
class Calibrator;

class NonlinearCalibrator
{
    public:
        virtual void calibrate(Calibrator*) = 0;
        virtual ~NonlinearCalibrator() {}
        virtual void write_info(std::ostream &os) {}
};

#endif/*_NONLINEARCALIBRATOR_H_*/


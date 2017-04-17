#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <cmath>
#include <ctime>
#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double max, double min, double Kp, double Ki, double Kd );
        ~PIDImpl();
        double calculate( double setpoint, double pv );
        void setKp(double newKp);
        void setKi(double newKi);
        void setKd(double newKd);
        void setMin(double newMin);
        void setMax(double newMax);
        void reset();
        double lastError();

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _integral;
        time_t _time_last;
        time_t _time_now;
        bool _windup;
        double _pv_last;
        double _error;
        double _Pout;
        double _Iout;
        double _Dout;
};


PID::PID( double max, double min, double Kp, double Ki, double Kd )
{
    pimpl = new PIDImpl(max,min,Kp,Ki,Kd);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID()
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double max, double min, double Kp, double Ki, double Kd ) :
    _dt(0),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _integral(0),
    _time_last(clock()),
    _time_now(0),
    _windup(false),
    _pv_last(0),
    _error(0),
    _Pout(0),
    _Iout(0),
    _Dout(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    _time_now = clock();

    // Calculate error
    _error = setpoint - pv;

    // Proportional term
    _Pout = _Kp * _error;

    // Integral term
    _dt = double(_time_now - _time_last)/CLOCKS_PER_SEC;

    if (_windup == false) {
    _integral += _error * _dt;
    }

    double _Iout = _Ki * _integral;

    // Derivative term
    if (_dt > 0){
    double derivative = (pv - _pv_last) / (_dt*50);
    _Dout = _Kd * derivative;
    }
    else{
    _Dout = 0;
    }

    // Calculate total output
    double output = _Pout + _Iout + _Dout;

    // Restrict to max/min
    if( output > _max ){
        output = _max;
        _windup = true;}
    else if( output < _min ){
        output = _min;
        _windup = true;}
    else{
        _windup = false;}

    // Save error to previous error
    _pv_last = pv;
    _time_last = _time_now;

    return output;
}

void PIDImpl::setKp(double newKp){
    _Kp = newKp;
}
void PIDImpl::setKi(double newKi){
    _Ki = newKi;
}
void PIDImpl::setKd(double newKd){
    _Kd = newKd;
}
void PIDImpl::setMin(double newMin){
    _min = newMin;
}
void PIDImpl::setMax(double newMax){
    _max = newMax;
}
double PIDImpl::lastError(){
    return _error;
}
void PIDImpl::reset(){
    _integral = 0;
}

void PID::setKp(double newKp){
    pimpl->setKp(newKp);
}
void PID::setKi(double newKi){
    pimpl->setKi(newKi);
}
void PID::setKd(double newKd){
    pimpl->setKd(newKd);
}
void PID::setMin(double newMin){
    pimpl->setMin(newMin);
}
void PID::setMax(double newMax){
    pimpl->setMax(newMax);
}
double PID::lastError(){
    return pimpl->lastError();
}
void PID::reset(){
    pimpl->reset();
}


PIDImpl::~PIDImpl()
{
}

#endif

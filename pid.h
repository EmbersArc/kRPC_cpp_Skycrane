#ifndef _PID_H_
#define _PID_H_

class PIDImpl;
class PID
{
    public:

        PID( double max, double min, double Kp, double Ki, double Kd );
        void setKp(double newKp);
        void setKi(double newKi);
        void setKd(double newKd);
        void setMin(double newMin);
        void setMax(double newMax);
        void reset();
        double lastError();

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};


#endif

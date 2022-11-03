#ifndef PID_H
#define PID_H

namespace drone_control{

class PID
{   
    public:

    PID();
    ~PID();

    void SetGainParameters(double params[5]);
    void ReSet();
    double ComputeCorrection(double _cmd, double _pos, double _loopTime);
    double ComputeCorrectionLimit(double _cmd, double _pos, double _loopTime);

    private:

    double G;
    double Kp, Kd, Ki, Ki_max= 0;
    double speedCmd = 0;

    double posErrorPrev = 0;
    double posIntegrator = 0;

    double error = 0;
    double errorPrev = 0;
    double integrator = 0;

};


}

#endif
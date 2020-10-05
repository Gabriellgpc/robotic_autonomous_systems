#include "pid.hpp"

#include <ctime>

PID::PID(const double _kp, double _ki, double _kd):
kp(_kp),
ki(_ki),
kd(_ki),
cumError(0.0),
prevValue(0.0),
prevTime(0.0),
inited(false)
{

}
void PID::reset()
{
    cumError = 0.0;
    prevValue = 0.0,
    prevTime = 0.0;
    inited = false;
}

void PID::update(const double _kp, double _ki, double _kd)
{
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

// para diminuir as oscilações na parcela derivativa
// uso aqui a derivada da saida, ao em vez da derivada do error
// ilustrações sobre disso:
// https://controlguru.com/pid-control-and-derivative-on-measurement/
// https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html
double PID::step(const double ref, const double curr)
{
    double error = ref - curr;
    double dt;
    
    struct timespec currTime_spec;
	clock_gettime(CLOCK_MONOTONIC, &currTime_spec);
    double currTime = (currTime_spec.tv_sec + currTime_spec.tv_nsec*1e-9); 

    double dError = 0;
    if (!inited){
        inited = true;
    } else {
        dt = currTime - prevTime;
        cumError += error * dt;
        dError = (curr - prevValue) / dt;
    }

    double pTerm = kp * error;
    double iTerm = ki * cumError;
    double dTerm = -kd * dError;

    prevTime  = currTime;
    prevValue = curr;

    return pTerm + iTerm + dTerm;
}
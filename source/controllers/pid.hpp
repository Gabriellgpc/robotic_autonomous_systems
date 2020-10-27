#pragma once

class PID{
public:
    PID(const double _kp, double _ki, double _kd);
    ~PID(){reset();}
    void update(const double _kp, double _ki, double _kd);
    // recebe a referencia e o valor atual da saida
    // retorna o sinal de controle
    double step(const double error);
    void reset();
private:
    double kp;
    double ki;
    double kd;
    
    double integral;
    double deriv;
    
    bool inited = false;
    double prevTime; //[s]
    double prevError;
};
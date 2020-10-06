#pragma once
#include "pid.hpp"

//distancia minima [m], utilizada como condicao de parada para o controlador de posicao
#define MIN_DIST 0.01 //[m]

class PositionController{
public:
    PositionController(const double lin_Kp,const double lin_Ki,const double lin_Kd,
                       const double ang_Kp,const double ang_Ki,const double ang_Kd);
                            
    /* input: (referencial global)
    * x_ref:    coordenada x da posição de referência [m]
    * y_ref:    coordenada y da posição de referência [m]
    * x_curr:   coordenada x atual [m]
    * y_curr:   coordenada y atual [m]
    * th_curr:  orientação atual no plano x-y [rad]
    *output: 
    * u_v:      velocidade linear [m/s]
    * u_w:      velocidade angular[rad/s] 
    * lin_error:  erro linear [m]  (util para debug) 
    * ang_error:  erro angular[rad](util para debug) 
    * return: true ao chegar na referencia, false caso contrario
    */
    bool step(const double x_ref, const double y_ref, 
              const double x_curr, const double y_curr, const double th_curr,
              double &u_v, double &u_w,
              double &lin_error, double &ang_error);
    
    //reset to PID controllers
    void reset();

    //update the PID's parameters
    void update(const double lin_Kp,const double lin_Ki,const double lin_Kd,
                const double ang_Kp,const double ang_Ki,const double ang_Kd);
private:
    bool _closeEnough(const double dl);

    PID lin_controller;
    PID ang_controller;
};
#pragma once
#include "pid.hpp"

// Seguidor de Caminho [Samson]
// Descrição do robô em relação ao caminho:
// – baseada em um referencial Serret-Frenet
// – vetor tangente, vetor normal e vetor binormal
// – O referencial S-F é localizado no ponto da
// projeção ortonormal do robô sobre o caminho.
// – O referencial S-F move-se ao longo do
// caminho.
class PathFollowController{
public:
    PathFollowController(const double K_ang,const double K_lin,const double path_coef[]);

    bool step(const double x_curr, const double y_curr, const double th_curr,
              double &u_v, double &u_w,
              double &lin_error, double &ang_error);

    //update the  parameters
    void update(const double K_ang,const double K_lin,const double path_coef[]);

    bool closestPoint(const double &x_curr, const double &y_curr, 
                      double &x, double &y, double &th, double &k, double &mindist);
private:
    double K_ang, K_lin;//ganho linear e ganho linear respectivamente
    double coef[8];     //parametros do caminho (polinomio de grau 3)
    double prev_lambda;
};


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
    PID lin_controller;
    PID ang_controller;
};


// DFL - Dynamic Feedback Linearization (Novel, et al 1995)
// envolve Realimentação PD e Compensação do Modelo
// Não Linear.
class TrajController{
public:
    TrajController(const double _Kd, const double _Kp);
    ~TrajController(){reset();}
    void setTrajectory(const double pathCoef[], const double tmax);
    /*
    * input: 
    * currConfig[3] : {x[m], y[m], theta[rad]}, configuração atual do robo
    * currVelocity  : velocidade linear do robo com relação ao mundo [m/s]
    * output:
    * v : velocidade linear que deve ser aplicada no robo [m/s]
    * w : velocidade angular que deve ser aplicada no robo [rad/s]
    */
    bool step(const double currConfig[3], const double currVelocity, double &v, double &w);
    
    //controller reset
    void reset();

    // Perfil de Velocidade Cosenoidal
    // v(t) = [1 – cos(2pi*t/tmax )].vmax/2
    double speedProfile_cos(const double t, const double tmax, const double vmax);

    double speedProfile_cos_derivate(const double t, const double tmax, const double vmax);

    void trajectoryGenerator(const double coef[], const double t, double &l,
                             double &x, double &y,
                             double &dx, double &dy,
                             double &d2x, double &d2y);
private:
    double Kd, Kp;
    bool inited = false;
    bool haveTraj=false;
    double coef[8]; //parametros que definem o caminho (polinomio de grau 3)
    double tmax;
    // bool haveTrajectory = false;
};
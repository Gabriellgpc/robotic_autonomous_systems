#include "kinematicsControllers.hpp"


#include "utils.hpp"
#include <cmath>
#include <cstring>
#include <ctime>

#include <cstdio>

/************************************************************************************************/

PathFollowController::PathFollowController(const double K_ang,
                                           const double K_lin,
                                           const double path_coef[]):
K_ang(K_ang),
K_lin(K_lin),
prev_lambda(0)
{
    std::memcpy(coef, path_coef, 8*sizeof(double));
}

void PathFollowController::update(const double K_ang,
                                  const double K_lin,
                                  const double path_coef[])
{
    this->K_ang = K_ang;
    this->K_lin = K_lin;
    this->prev_lambda = 0.0;
    std::memcpy(this->coef, path_coef, 8*sizeof(double));
}
bool PathFollowController::step(const double x_curr, const double y_curr, const double th_curr,
                                double &v, double &w,
                                double &lin_error, double &ang_error)
{
    static double u;
    static double x, y, th, k;  //x, y, theta e curvatura (kappa) no ponto sobre a curva
    bool end = closestPoint(x_curr, y_curr, x, y, th, k, lin_error);
    ang_error = th_curr - th;

    u = -(K_ang*ang_error + K_lin*lin_error*v*sin(ang_error)/(ang_error + 0.001));
    w = u + k*v*cos(ang_error)/(1.0 - k*lin_error);
    
    if(end){
        v = 0;
        w = 0;
    }

    // printf("l = %.3lf|(xc:%.2lf, yc:%.2lf) | lin_error = %.2lf | ang_error = %.2lf\n", prev_lambda, x, y, lin_error, ang_error);
    return end;
}
// retorna o ponto mais proximo
// retorna true se "acabou o caminho" (lambda >= 1)
bool PathFollowController::closestPoint(const double &x_curr, const double &y_curr, 
                                        double &x, double &y, double &th, double &k,double &mindist)
{
    // N pontos
    static const double step = 1.0/200.0;
    static double lambda, x_diff, y_diff, dist;
    static float xp, yp, thp;    //x, y, theta no ponto sobre a curva

    mindist = 999999;

    for(lambda = this->prev_lambda; lambda <= 1.0; lambda+= step){
        poly3(coef, lambda, xp, yp, thp);    
        x_diff = xp - x_curr;  y_diff = yp - y_curr;      
        dist = sqrt(x_diff*x_diff + y_diff*y_diff);
        
        if(dist < mindist){
           x = xp;
           y = yp;
           th= thp;
           k = curvature(coef, lambda);
           this->prev_lambda = lambda;
           mindist = dist;     
        }
    }

    if(this->prev_lambda > 0.9)return true;
    return false;
}
/************************************************************************************************/

TrajController::TrajController(const double _Kd, const double _Kp):
Kd(_Kd),
Kp(_Kp),
inited(false),
haveTraj(false),
tmax(0)
{
    memset(coef, 0, sizeof(coef));
}
//controller reset
void TrajController::reset()
{   
    std::memset(coef, 0, 8*sizeof(double));    
    inited = false;
    haveTraj= false;
    tmax = 0;
}

void TrajController::setTrajectory(const double pathCoef[], const double tmax)
{
    reset();
    std::memcpy(coef, pathCoef, 8*sizeof(double));
    this->tmax = tmax;
    this->haveTraj=true;
}

// Perfil de Velocidade Cosenoidal
// v(t) = [1 – cos(2pi*t/tmax )].vmax/2
double TrajController::speedProfile_cos(const double t, const double tmax, const double vmax)
{
    return (1.0 - cos(2.0*M_PI*t/tmax))*vmax/2.0;
}

// v'(t) = [sen(2.t/t max )]..v max /t max
double TrajController::speedProfile_cos_derivate(const double t, const double tmax, const double vmax)
{
    return sin(2*M_PI*t/tmax)*M_PI*vmax/tmax;
}

/*
* input: 
* currConfig[3] : {x[m], y[m], theta[rad]}, configuração atual do robo
* currVelocity  : velocidade linear do robo com relação ao mundo [m/s]
* output:
* v : velocidade linear que deve ser aplicada no robo [m/s]
* w : velocidade angular que deve ser aplicada no robo [rad/s]
*/
bool TrajController::step(const double currConfig[3], 
                          const double currVelocity, 
                          double &v, double &w)
{
    if(!haveTraj)return true;

    static double L, l, vmax, dt, t;
    static double Dx,DDx,Dy,DDy, x, y, th, v_l, dv_l;   //variaveis da trajetoria
    static double ddxc, ddyc, dv, vc, wc;               //variaveis do controlador
    static double currTime, prevTime;
    static struct timespec currTime_spec;
	clock_gettime(CLOCK_MONOTONIC, &currTime_spec);    
    
    currTime = (currTime_spec.tv_sec + currTime_spec.tv_nsec*1e-9);

    if(!inited){
        l = 0;
        t = 0;
        vc= 0.01;
        Dx = coef[1];
        Dy = coef[5];
        //comprimento total do caminho => s(lambda = 1)
        prevTime = currTime;
        L = poly3Length(coef, 1.0);
        vmax = 2.0*L/tmax;
        inited = true;
    }
    dt = currTime - prevTime;
    t += dt;

    /**** Proxima config. da trajetoria ****/
    dv_l= speedProfile_cos_derivate(t,tmax, vmax);    
    v_l = speedProfile_cos(t, tmax, vmax);
    //lambda(t)
    l += v_l*dt/sqrt( Dx*Dx + Dy*Dy );
    //computing x(l),y(l),th(l), dx, dy 
    double l2 = l*l, l3 = l2*l;  
    x  = coef[0] + coef[1]*l + coef[2]*l2 + coef[3]*l3;
    y  = coef[4] + coef[5]*l + coef[6]*l2 + coef[7]*l3;
    Dx = coef[1] + 2.0*coef[2]*l + 3.0*coef[3]*l2;
    Dy = coef[5] + 2.0*coef[6]*l + 3.0*coef[7]*l2;
    th = atan2(Dy,Dx);
    //computing  d2x, d2y and W
    DDx = 2.0*coef[2] + 6.0*coef[3]*l;
    DDy = 2.0*coef[6] + 6.0*coef[7]*l;

    /**** Controle da Trajetoria ****/

    // Realimentação PD – Acelerações de Comando
    // x c '' = x * '' + K dx (x * '- x') + K px (x * - x)
    // y c '' = y * '' + K dy (y * '- y') + K py (y * - y)
    static double dx_curr, dy_curr, x_curr, y_curr, th_curr;
    static double dx, dy, ddx, ddy;
    x_curr  = currConfig[0];
    y_curr  = currConfig[1];
    th_curr = currConfig[2];
    dx_curr = currVelocity*cos(th_curr);    
    dy_curr = currVelocity*sin(th_curr);

    dx = v_l*cos(th);
    dy = v_l*sin(th);
    ddx= dv_l*cos(th);
    ddy= dv_l*sin(th);
    
    // v_l*cos(th),v_l*sin(th), dv_l*cos(th),dv_l*sin(th)
    ddxc = ddx + Kd*(dx - dx_curr) + Kp*(x - x_curr);
    ddyc = ddy + Kd*(dy - dy_curr) + Kp*(y - y_curr);
    
    // Compensação do Modelo Não Linear
    // printf("l = %.2f : (x*:%.2f , y*: %.2f) | (x*':%.2f , y*': %.2f) | (x*'':%.2f , y*'': %.2f)\n", l, x,y, dx,dy, ddx,ddy);
    dv = ddxc*cos(th_curr) + ddyc*sin(th_curr);
    wc = (ddyc*cos(th_curr) - ddxc*sin(th_curr))/(currVelocity + 0.01);
    
    // integração - Velocidade linear de comando
    vc += dv*dt;
    
    // output
    v = vc;
    w = wc;

    prevTime = currTime;
    //tempo maximo atingido
    if(t >= this->tmax){
        v = 0.0;
        w = 0.0;
        return true;
    }
    // printf("t = %0.2f  | lambda = %0.2f | v* = %0.2f | v = %0.2f | w = %0.2f\n", t, l, v_l, v, w);
    
    return false;
}


/************************************************************************************************/
PositionController::PositionController(const double lin_Kp,const double lin_Ki,const double lin_Kd,
                                       const double ang_Kp,const double ang_Ki,const double ang_Kd):
                                       lin_controller(lin_Kp, lin_Ki, lin_Kd),
                                       ang_controller(ang_Kp, ang_Ki, ang_Kd)
{
    
}
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
* return: true ao chegar no destino
*/

//distancia minima [m], utilizada como condicao de parada para o controlador de posicao
#define MIN_DIST 0.10 //[m]
bool PositionController::step(const double x_ref, const double y_ref, 
                              const double x_curr, const double y_curr, const double th_curr,
                              double &u_v, double &u_w,
                              double &lin_error, double &ang_error)
{
    static double delta_x, delta_y;
    delta_x = x_ref - x_curr;
    delta_y = y_ref - y_curr;
    
    ang_error = atan2(delta_y, delta_x) - th_curr;
    lin_error = sqrt( delta_x*delta_x + delta_y*delta_y ) * cos(ang_error);

    if(fabs(lin_error) <= MIN_DIST){
        u_v = 0.0;
        u_w = 0.0;
        this->reset();
        return true;
    }
    u_v = lin_controller.step(lin_error);
    u_w = ang_controller.step(ang_error);
    
    return false;
}
void PositionController::reset()
{
    lin_controller.reset();
    ang_controller.reset();
}     

void PositionController::update(const double lin_Kp,const double lin_Ki,const double lin_Kd,
                                const double ang_Kp,const double ang_Ki,const double ang_Kd)
{
    lin_controller.update(lin_Kp, lin_Ki, lin_Kd);
    ang_controller.update(ang_Kp, ang_Ki, ang_Kd);
}
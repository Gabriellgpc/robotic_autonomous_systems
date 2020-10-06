#include "kinematicsControllers.hpp"

#include <cmath>

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
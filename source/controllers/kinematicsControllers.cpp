#include "kinematicsControllers.hpp"

#include <utils.hpp>

#include <cmath>
#include <cstring>
#include <ctime>

#include <cstdio>

/************************************************************************************************/
PathFollowController::PathFollowController() : _K_ang(0),
                                               _K_lin(0),
                                               _prev_point(),
                                               _points()
{
}
PathFollowController::PathFollowController(const double K_ang, const double K_lin, const std::list<Config> &points) : _K_ang(K_ang),
                                                                                                                      _K_lin(K_lin),
                                                                                                                      _points(points)
{
    _prev_point = _points.begin();
}
PathFollowController::PathFollowController(const double K_ang, const double K_lin) : _K_ang(K_ang),
                                                                                     _K_lin(K_lin),
                                                                                     _points()
{
    _prev_point = _points.begin();
}
PathFollowController::~PathFollowController()
{
    _K_ang = 0;
    _K_lin = 0;
    _points.clear();
    _prev_point = _points.begin();
}

bool PathFollowController::step(const Config &curr_q,
                                double &v, double &w,
                                Config &ref_q,
                                double &lin_error, double &ang_error)
{
    static double u;
    static double k; //x, y, theta e curvatura (kappa) no ponto sobre a curva

    closestPoint(curr_q, ref_q, lin_error, k);
    ang_error = curr_q.get_theta() - ref_q.get_theta();

    u = -(_K_ang * ang_error + _K_lin * lin_error * v * sin(ang_error) / (ang_error + 0.0001));
    w = u; // + k * v * cos(ang_error) / (1.0 - k * lin_error);

    if ((_points.back().get_pos() - curr_q.get_pos()).norm() <= 15.0e-2)
    {
        v = 0;
        w = 0;
        return true;
    }
    return false;
}
//update the  parameters
void PathFollowController::update(const double K_ang, const double K_lin, const std::list<Config> &points)
{
    this->_K_ang = K_ang;
    this->_K_lin = K_lin;
    this->_points = points;
    this->_prev_point = _points.begin();
}
void PathFollowController::closestPoint(const Config &q, Config &ref, double &mindist, double &kappa)
{
    // N pontos
    static double delta_x, delta_y, dist;
    static double min_dist_used = 0.01;
    static double prev_orientation;
    Vector2D normal, point_to_q; //vetor normal ao segmento de reta; vetor saindo do waypoint ate a posicao atual

    mindist = 999999;

    for (auto p = _prev_point; p != _points.end(); p++)
    {
        delta_x = q.x() - p->x();
        delta_y = q.y() - p->y();
        dist = sqrt(delta_x * delta_x + delta_y * delta_y);

        if ((dist <= mindist) && (dist >= 1.0e-6))
        {
            if (q.get_pos() == p->get_pos())
                continue;
            ref = *p;
            if ((ref.get_pos() - _prev_point->get_pos()).norm() > 1e-6)
                ref.theta() = atan2(ref.y() - _prev_point->y(), ref.x() - _prev_point->x());
            else
                ref.theta() = atan2((++p)->get_pos().y() - ref.y(), (++p)->get_pos().y() - ref.x());

            // WARNING: alterar para curvatura de uma função suave que interpole o ponto anteriro com o ponto atual
            kappa = (ref.theta() - prev_orientation) / ((p->get_pos() - _prev_point->get_pos()).norm() + 0.001);

            mindist = dist;
            _prev_point = p;
            prev_orientation = ref.theta();
        }
    }
}

/************************************************************************************************/

TrajController::TrajController(const double _Kd, const double _Kp) : Kd(_Kd),
                                                                     Kp(_Kp),
                                                                     inited(false),
                                                                     haveTraj(false),
                                                                     vmax(0.0)
{
    memset(coef, 0, sizeof(coef));
}
//controller reset
void TrajController::reset()
{
    std::memset(coef, 0, 8 * sizeof(double));
    inited = false;
    haveTraj = false;
    vmax = 0.0;
}

void TrajController::setTrajectory(const double pathCoef[], const double vmax)
{
    reset();
    std::memcpy(coef, pathCoef, 8 * sizeof(double));
    this->vmax = vmax;
    this->haveTraj = true;
}

// Perfil de Velocidade Cosenoidal
// v(t) = [1 – cos(2pi*t/tmax )].vmax/2
double TrajController::speedProfile_cos(const double t, const double tmax, const double vmax)
{
    return (1.0 - cos(2.0 * M_PIf64 * t / tmax)) * vmax / 2.0;
}

// v'(t) = [sen(2.t/t max )]..v max /t max
double TrajController::speedProfile_cos_derivate(const double t, const double tmax, const double vmax)
{
    return sin(2.0 * M_PIf64 * t / tmax) * M_PIf64 * vmax / tmax;
}

/*
* input: 
* currConfig[3] : {x[m], y[m], theta[rad]}, configuração atual do robo
* currVelocity  : velocidade linear do robo com relação ao mundo [m/s]
* output:
* v : velocidade linear que deve ser aplicada no robo [m/s]
* w : velocidade angular que deve ser aplicada no robo [rad/s]
*/
bool TrajController::step(const double currConfig[],
                          double &x, double &y, double &v_l, double &dv_l, double &wc,
                          double &v, double &w)
{
    if (!haveTraj)
        return true;

#define R 4.0

    static double L, l, tmax, dt, t;
    static double Dx, DDx, Dy, DDy, th; //variaveis da trajetoria
    static double ddxc, ddyc, vc, dv;   //variaveis do controlador
    static double currTime, prevTime;
    static struct timespec currTime_spec;
    static double integral_vel = 0.0;
    clock_gettime(CLOCK_MONOTONIC, &currTime_spec);

    currTime = (currTime_spec.tv_sec + currTime_spec.tv_nsec * 1e-9);

    if (!inited)
    {
        l = 0.0;
        t = 0.0;
        vc = 0.01;
        // poly 3
        Dx = coef[1] + 2.0 * coef[2] * l + 3.0 * coef[3] * l * l;
        Dy = coef[5] + 2.0 * coef[6] * l + 3.0 * coef[7] * l * l;
        L = poly3Length(coef, 1.0);

        //comprimento total do caminho => s(lambda = 1)
        prevTime = currTime;
        tmax = 2.0 * L / vmax;
        inited = true;
    }

    /**** Proxima config. da trajetoria ****/
    dt = currTime - prevTime;
    t += dt;
    prevTime = currTime;

    dv_l = speedProfile_cos_derivate(t, tmax, vmax);
    v_l = speedProfile_cos(t, tmax, vmax);

    //lambda(t)
    double dl = v_l * dt / sqrt(Dx * Dx + Dy * Dy);
    l += dl;
    //computing x(l),y(l),th(l), Dx, Dy, DDx and DDy
    //poly 3
    double l2 = l * l, l3 = l2 * l;
    x = coef[0] + coef[1] * l + coef[2] * l2 + coef[3] * l3;
    y = coef[4] + coef[5] * l + coef[6] * l2 + coef[7] * l3;
    Dx = coef[1] + 2.0 * coef[2] * l + 3.0 * coef[3] * l2;
    Dy = coef[5] + 2.0 * coef[6] * l + 3.0 * coef[7] * l2;
    th = atan2(Dy, Dx);
    //computing  d2x, d2y and W
    DDx = 2.0 * coef[2] + 6.0 * coef[3] * l;
    DDy = 2.0 * coef[6] + 6.0 * coef[7] * l;
    /**** Controle da Trajetoria ****/

    // Realimentação PD – Acelerações de Comando
    // x c '' = x * '' + K dx (x * '- x') + K px (x * - x)
    // y c '' = y * '' + K dy (y * '- y') + K py (y * - y)
    static double dx_curr, dy_curr, x_curr, y_curr, th_curr, speed_curr;
    static double dx, dy, ddx, ddy;
    static double int_vel_robo = 0.0;
    x_curr = currConfig[0];
    y_curr = currConfig[1];
    th_curr = currConfig[2];
    dx_curr = currConfig[3];
    dy_curr = currConfig[4];
    speed_curr = sqrt(dx_curr * dx_curr + dy_curr * dy_curr);
    int_vel_robo += speed_curr * dt;

    dx = v_l * cos(th);
    dy = v_l * sin(th);
    ddx = dv_l * cos(th);
    ddy = dv_l * sin(th);

    // v_l*cos(th),v_l*sin(th), dv_l*cos(th),dv_l*sin(th)
    ddxc = ddx + Kd * (dx - dx_curr) + Kp * (x - x_curr);
    ddyc = ddy + Kd * (dy - dy_curr) + Kp * (y - y_curr);

    // Compensação do Modelo Não Linear
    dv = ddxc * cos(th_curr) + ddyc * sin(th_curr);
    wc = (ddyc * cos(th_curr) - ddxc * sin(th_curr)) / (speed_curr + 0.01);

    // integração - Velocidade linear de comando
    vc += dv * dt;

    // output
    v = vc;
    w = wc;

    //fim da trajetoria
    if ((t >= tmax) || (l >= 1.0) || (integral_vel >= (L - 0.01)))
    {
        printf("dt =  %.4lf | t = %.4lf  | tmax = %0.4lf | dl = %.4lf | l = %.4lf | integral(v(t)) = %.4lf | integral(v_robot(t)) = %.4lf |L = %.4lf\n", dt, t, tmax, dl, l, integral_vel, int_vel_robo, L);
        v = 0.0;
        w = 0.0;
        return true;
    }

    return false;
}

/************************************************************************************************/
PositionController::PositionController(const double lin_Kp, const double lin_Ki, const double lin_Kd,
                                       const double ang_Kp, const double ang_Ki, const double ang_Kd) : lin_controller(lin_Kp, lin_Ki, lin_Kd),
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
#define MIN_DIST 0.09 //[m]
bool PositionController::step(const Config &q_ref, const Config &q_curr)
{
    static double delta_x, delta_y;
    delta_x = q_ref.x() - q_curr.x();
    delta_y = q_ref.y() - q_curr.y();

    prev_ang_error = atan2(delta_y, delta_x) - q_curr.theta();
    prev_lin_error = sqrt(delta_x * delta_x + delta_y * delta_y) * cos(prev_ang_error);

    if (fabs(prev_lin_error) <= MIN_DIST)
    {
        prev_v = 0.0;
        prev_w = 0.0;
        this->reset();
        return true;
    }
    prev_v = lin_controller.step(prev_lin_error);
    prev_w = ang_controller.step(prev_ang_error);

    return false;
}
void PositionController::reset()
{
    lin_controller.reset();
    ang_controller.reset();
}

void PositionController::update(const double lin_Kp, const double lin_Ki, const double lin_Kd,
                                const double ang_Kp, const double ang_Ki, const double ang_Kd)
{
    lin_controller.update(lin_Kp, lin_Ki, lin_Kd);
    ang_controller.update(ang_Kp, ang_Ki, ang_Kd);
}
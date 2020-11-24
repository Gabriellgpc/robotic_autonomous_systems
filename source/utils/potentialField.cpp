#include "potentialField.hpp"
#include <cmath>

PotentialField::PotentialField() : _configSpace(),
                                   _has_new_obj(false)
{
}
PotentialField::PotentialField(const World &w) : _configSpace(w),
                                                 _has_new_obj(false)
{
    _configSpace.compute_c_obstacles(N_SAMPLES);
}
PotentialField::~PotentialField()
{
    _has_new_obj = false;
}
//força respulsiva
//rho_0: distancia de influencia do campo respulsivo
//nhi: fator de ganho positivo
void PotentialField::add_obstacle(const Polygon2D &obj)
{
    _configSpace.add_obstacle(obj);
    _has_new_obj = true;
}
//retorna uma copia do espaço de configuração/trabalho atual
World PotentialField::get_space()
{
    return _configSpace;
}
void PotentialField::set_space(const World &w)
{
    _configSpace = w;
}

double PotentialField::config_dist(const Config &q1, const Config &q2, const double &robot_radius)
{
    double dist;
    double delta_x, delta_y;
    double delta_th = q2.theta() - q1.theta();

    if (delta_th >= 2.0 * M_PI)
        delta_th -= 2.0 * M_PI;
    if (delta_th < 0.0)
        delta_th += 2.0 * M_PI;

    delta_x = q2.x() - q1.x();
    delta_y = q2.y() - q1.y();

    dist = sqrt(delta_x * delta_x + delta_y * delta_y + delta_th * delta_th * robot_radius * robot_radius);

    return dist;
}

Config PotentialField::repulsive_force(const Polygon2D &obj, const Robot &robot)
{
    Config fr(0, 0, 0);
    Polygon2D c_obstacle = obj.work_to_config_space(robot.to_polygon2D());
    double min_dist = c_obstacle.min_radius() * 2.0;

    Vector2D robot_to_cb = c_obstacle.center() - robot.get_config().get_pos();
    double dist = robot_to_cb.norm();
    double intensity = pow(c_obstacle.area(), 3.0);
    double inv_min_dist = 1.0 / min_dist;
    double fr_scalar;

    if (dist > min_dist)
        return fr;

    fr_scalar = intensity * ((1.0 / dist) - inv_min_dist) * pow(1.0 / dist, 2.0);

    //vetor que aponta do centor do c_obstaculo para o robo, normalizado
    fr.x() = robot_to_cb.x() * fr_scalar;
    fr.y() = robot_to_cb.y() * fr_scalar;
    fr.theta() = 0.0;

    return fr;
}

std::list<Config> PotentialField::get_planned_path(const Config &q,
                                                   const Config &qf,
                                                   const double &R,
                                                   const double &ksi)
{
    _configSpace.compute_c_obstacles(N_SAMPLES);

    std::list<Config> plan;
    static const double min_dist_to_qf = 0.01;
    Vector2D f_xy(0, 0);
    Config f_result, f_atr, f_rp;
    double curr_config_dist = 99999.0;
    Config curr_q = q;
    Config prev_q = q;

    while (curr_config_dist >= min_dist_to_qf)
    {
        plan.push_back(curr_q);
        _configSpace.update_config(curr_q);

        //soma das forças repulsivas
        f_xy = Vector2D(0, 0);
        auto c_obstacles = _configSpace.get_cobstacles(N_SAMPLES);
        for (auto cb_it = c_obstacles.begin(); cb_it != c_obstacles.end(); cb_it++)
        {
            f_xy = f_xy + repulsive_force(*cb_it, _configSpace.get_robot()).get_pos();
        }
        f_rp = Config(f_xy, 0.0);

        //força atrativa no target (qf)
        f_atr = Config(curr_q.x() - qf.x(),
                       curr_q.y() - qf.y(),
                       curr_q.theta() - qf.theta());

        //força resultante
        f_result.x() = f_rp.x() + f_atr.x();
        f_result.y() = f_rp.y() + f_atr.y();
        f_result.theta() = f_rp.theta() + f_atr.theta();

        //proxima configuração
        curr_q.x() = curr_q.x() - ksi * f_result.x();
        curr_q.y() = curr_q.y() - ksi * f_result.y();
        curr_q.theta() = curr_q.theta() - (ksi / R) * f_result.theta();

        curr_config_dist = config_dist(curr_q, qf, R);
        if (prev_q == curr_q)
        {
            std::cout << "Minimo local!\n";
            break;
        }
        prev_q = curr_q;
    }
    return plan;
}
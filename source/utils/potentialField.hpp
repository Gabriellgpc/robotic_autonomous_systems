#pragma once
#include "configSpaceTools.hpp"
#include <list>

static const unsigned int N_SAMPLES = 200;

class PotentialField
{
public:
    PotentialField();
    PotentialField(const World & w);
    ~PotentialField();

    //força respulsiva
    //rho_0: distancia de influencia do campo respulsivo
    //nhi: fator de ganho positivo
    void add_obstacle(const Polygon2D &obj);

    //retorna uma copia do espaço de configuração/trabalho atual
    World get_space();
    void  set_space(const World &w);

    double config_dist(const Config &q1, const Config &q2, const double &robot_radius);

    Config repulsive_force(const Polygon2D &obj, const Robot &robot);
    std::list<Config> get_planned_path(const Config &q, const Config &qf, const double &R, const double &ksi);
private:    
    World _configSpace;
    bool  _has_new_obj = false;    
};
#include "occupating_grid.hpp"
#include <cmath>
#include <string>
#include <fstream>

ProximitySensorInfo::ProximitySensorInfo(const float &min_range, 
                                         const float &max_range, 
                                         const float &opening, 
                                         const float &angle,
                                         const float &e)
{
    this->range[0] = min_range;
    this->range[1] = max_range;
    this->opening_angle = opening;
    this->theta = angle;
    this->error = e;

    this->sensor_measure_dist = -1.0;
    //ponto no plano x,y do mundo / localização do ponto detectado
    this->sensor_measure_point= Vector2D(999999,999999);
}
ProximitySensorInfo::~ProximitySensorInfo()
{
    // nothing to do

}

//atualize measure_dist com um valor negativo para indicar que não ha dados de medição
void ProximitySensorInfo::updateMeasure(const float& measure_dist, const Vector2D & measure_point)
{
    this->sensor_measure_dist = measure_dist;
    this->sensor_measure_point= measure_point;
}

/*******************************************************************************************/

//grade com width e height composta de celulas quadradas com size_cell de lado
OccupationGrid::OccupationGrid(const float &width, 
                               const float &height, 
                               const float& size_cell):
_width(width),
_height(height),
_size_cell(size_cell)
{   
    int map_height   = static_cast<int>(round(height/size_cell));
    int map_width    = static_cast<int>(round(width/size_cell));
    this->map = std::vector<OccupationGridCell>(map_height*map_width);

    int xi,yi;
    for(int i = 0; i < this->map.size(); i++)
    {   
        xi = i % map_width;
        yi = floor(i / map_width);
        //origem do sistema fica no canto superior esquerdo. 
        // y crescente para "baixo" e x para a "direita"
        map[i].pos = Vector2D(xi * width/map_width, yi*height/map_height);
        //colocando o sistema no centro e y crescente para "cima"
        map[i].pos = map[i].pos.rotation(M_PI);
        map[i].pos = map[i].pos + Vector2D(width/2.0,height/2.0);
        map[i].l   = l_0;
        map[i].p   = inv_log_odd(l_0);
    }
}

OccupationGrid::~OccupationGrid()
{

}

void OccupationGrid::update(const ProximitySensorInfo& sensor, const Config & q)
{
    static float r;
    for(int i = 0; i < this->map.size(); i++)
    {   
        map[i].l += _inverse_model(map[i], sensor, q) - l_0;
        map[i].p = inv_log_odd(map[i].l);
    }
}

double OccupationGrid::_inverse_model(const OccupationGridCell &mi, const ProximitySensorInfo &z, const Config& q)
{
    static float r;
    static float phi;
    r = (mi.pos - q.get_pos()).norm();
    phi = atan2(mi.pos.y() - q.y(), mi.pos.x() - q.x()) - (q.theta() + z.get_ori());
    phi = fabs(phi);

    if( (z.zt() < 0) && (phi < z.ang_range()/2.0) && (r < z.max()))
        return l_L;

    if( (r > fmin(z.max(), z.zt() + z.e()/2.0)) ||  (phi > z.ang_range()/2.0 ) )
        return l_0;

    if( (z.zt() < z.max()) &&  (fabs(r - z.zt()) < z.e()/2.0) && ( phi < z.ang_range()/2.0 ))
        return l_oc;

    if( (r <= (z.zt() - z.e()/2.0)) && (phi < z.ang_range()/2.0) )
        return l_L;
    
    return l_0;
}

std::istream& OccupationGridCell::load_from_stream(std::istream &I)
{
    I >> this->pos.x();
    I >> this->pos.y();
    I >> this->l;
    I >> this->p;

    return I;
}
std::ostream& OccupationGridCell::save_to_stream(std::ostream &O)
{
    O <<'\n'<< this->pos.x(); 
    O <<' ' << this->pos.y();
    O <<' ' << this->l;
    O <<' ' << this->p;

    return O;
}

void OccupationGrid::save_to_file(const std::string file)
{
    std::ofstream out(file);

    if(out.is_open() == false)
    {
        std::cerr << "Falha ao salvar a Grade de Ocupação em arquivo\n";
        return;
    }

    out << map.size() << '\n';
    for(int i = 0; i < map.size(); i++)
        map[i].save_to_stream(out);
    out.close();
}
void OccupationGrid::load_from_file(const std::string file)
{
    std::ifstream in(file);
    std::size_t ncells;
    OccupationGridCell cell_tmp;

    if(in.is_open() == false)
    {
        std::cerr << "Falha ao ler a Grade de Ocupação do arquivo\n";
        return;
    }

    in >> ncells;

    map.resize(ncells);
    for(int i = 0; i < map.size(); i++)
    {
        cell_tmp.load_from_stream(in);
        map[i] = cell_tmp;
    }

    in.close();
}

double log_odd(const double &p)
{
    return log( p/(1 - p) );
}
double inv_log_odd(const double &l)
{
    return 1.0 - 1.0/(1.0 + exp(l));
}
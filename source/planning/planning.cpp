#include "planning.hpp"
#include <cmath>
#include <limits> // std::numeric_limits

RegularGrid::RegularGrid(const Vector2D &center,
                         const float &width,
                         const float &height,
                         const float &size_cell):
_width(width),
_height(height),
_size_cell(size_cell)
{
    int grid_height = static_cast<int>(round(height / size_cell));
    int grid_width = static_cast<int>(round(width / size_cell));
    this->_regularGrid = std::vector<CellGrid>(grid_height * grid_width);

    int xi, yi;
    for (int i = 0; i < this->_regularGrid.size(); i++)
    {
        xi = i % grid_width;
        yi = floor(i / grid_width);
        //origem do sistema fica no canto superior esquerdo.
        // y crescente para "baixo" e x para a "direita"
        _regularGrid[i].pos = Vector2D(xi * width / grid_width, yi * height / grid_height);
        //colocando o sistema no centro e y crescente para "cima"
        _regularGrid[i].pos = _regularGrid[i].pos.rotation(M_PI);
        _regularGrid[i].pos = _regularGrid[i].pos + Vector2D(width / 2.0, height / 2.0) + center;
        _regularGrid[i].value = 0;
        _regularGrid[i].free  = false;
    }
}

// RegularGrid OccupationGrid2RegularGrid(const OccupationGrid &OG, const Robot &robot);
RegularGrid manhattan(const OccupationGrid &OG,
                      const Config &qf,
                      const Robot &robot,
                      const double &threshold)
{
    RegularGrid RG( Vector2D(0,0), OG.get_width(), OG.get_height(), OG.get_size_cell());
    double robot_radius = robot.get_shape().min_radius();
    uint32_t M = std::numeric_limits<uint32_t>::max();
    
    double mindist = std::numeric_limits<double>::max();
    uint32_t qf_i  = 0;

    //Preenche a grade regular com M nas celulas ocupadas e -M nas livres
    for (std::size_t i = 0; i < OG.size(); i++)
    {   
        //testa se é uma região livre
        if(OG[i].p <= threshold)
        {
            //aumentar a area (passando de espaço de trabalho para espaço de config)
            for (std::size_t j = 0; j < OG.size(); j++)
            {   
                //se for uma celula ocupada
                if(OG[j].p > threshold)
                {
                    //verifica se a possivel celula livre (i) esta distante suficiente da celula ocupada (j)
                    //se estiver perto: considerar celular i como ocupada também
                    //caso contrario: considerar celula i livre
                    if(( OG[j].pos - RG[i].pos).norm() < robot_radius)
                    {
                        RG[i].value = M;
                        RG[i].free  = false;
                    }else{
                        RG[i].value = -M;
                        RG[i].free  = true;
                    }
                }
            }
        }else{//regiao ocupada
            RG[i].value = M;
            RG[i].free  = false;
        }

        if((RG[i].pos - qf.get_pos()).norm() < mindist && RG[i].free)
        {
            mindist = (RG[i].pos - qf.get_pos()).norm();
            qf_i = i;
        }

    }
    //qf = 0. Minimo global
    RG[qf_i].value = 0;


    return RG;
}
std::list<Config> depthFirst(const RegularGrid &grid,
                             const Config &qi,
                             const Config &qf)
{
}
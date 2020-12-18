#include "planning.hpp"
#include <cmath>
#include <queue>
#include <stack>
#include <cstring> //memset

RegularGrid::RegularGrid(const Vector2D &center,
                         const float &width,
                         const float &height,
                         const float &size_cell) : _size_cell(size_cell)
{
    _height = static_cast<int>(round(height / size_cell));
    _width = static_cast<int>(round(width / size_cell));
    this->_regularGrid = std::vector<CellGrid>(_height * _width);

    int xi, yi;
    for (int i = 0; i < this->_regularGrid.size(); i++)
    {
        xi = i % _width;
        yi = floor(i / _width);
        //origem do sistema fica no canto superior esquerdo.
        // y crescente para "baixo" e x para a "direita"
        _regularGrid[i].pos = Vector2D(xi * width / _width, yi * height / _height);
        //colocando o sistema no centro e y crescente para "cima"
        _regularGrid[i].pos = _regularGrid[i].pos.rotation(M_PI);
        _regularGrid[i].pos = _regularGrid[i].pos + Vector2D(width / 2.0, height / 2.0) + center;

        _regularGrid[i].value = 0;

        _regularGrid[i].free = false;

        _regularGrid[i].i = i;
    }
}

std::list<CellGrid> RegularGrid::get_4neighbors(const uint32_t &i)const
{
#define toIndex(x, y) ((x) + (y)*_width)
    static std::list<CellGrid> l;
    static uint32_t x, y;

    l.clear();

    x = i % _width;
    y = floor(i / _width);

    if ((x - 1) < _width)
        l.push_back(_regularGrid[toIndex(x - 1, y)]);
    if ((x + 1) < _width)
        l.push_back(_regularGrid[toIndex(x + 1, y)]);
    if ((y - 1) < _height)
        l.push_back(_regularGrid[toIndex(x, y - 1)]);
    if ((y + 1) < _height)
        l.push_back(_regularGrid[toIndex(x, y + 1)]);

    return l;
}

uint32_t RegularGrid::get_index(const Config &q) const
{
    uint32_t index = std::numeric_limits<uint32_t>::max();
    double mindist = std::numeric_limits<uint32_t>::max();

    for (std::size_t i = 0; i < _regularGrid.size(); i++)
    {
        if ((_regularGrid[i].pos - q.get_pos()).norm() < mindist)
        {
            mindist = (_regularGrid[i].pos - q.get_pos()).norm();
            index = i;
        }
    }

    return index;
}

// RegularGrid OccupationGrid2RegularGrid(const OccupationGrid &OG, const Robot &robot);
RegularGrid manhattan(const OccupationGrid &OG,
                      const Config &qf,
                      const Robot &robot,
                      const double &threshold)
{
    RegularGrid RG(Vector2D(0, 0), OG.get_width(), OG.get_height(), OG.get_size_cell());
    double robot_radius = robot.get_shape().min_radius();

    double mindist = std::numeric_limits<double>::max();
    uint32_t qf_i = 0;
    std::list<CellGrid> L;

    //Preenche a grade regular com M nas celulas ocupadas e -M nas livres
    for (std::size_t i = 0; i < OG.size(); i++)
    {
        //testa se é uma região livre
        if (OG[i].p <= threshold)
        {
            //aumentar a area (passando de espaço de trabalho para espaço de config)
            for (std::size_t j = 0; j < OG.size(); j++)
            {
                //se for uma celula ocupada
                if (OG[j].p > threshold)
                {
                    //verifica se a possivel celula livre (i) esta distante suficiente da celula ocupada (j)
                    //se estiver perto: considerar celular i como ocupada também
                    //caso contrario: considerar celula i livre
                    if ((OG[j].pos - RG[i].pos).norm() < robot_radius)
                    {
                        RG[i].value = M;
                        RG[i].free = false;
                    }
                    else
                    {
                        RG[i].value = -M;
                        RG[i].free = true;
                    }
                }
            }
        }
        else
        { //regiao ocupada
            RG[i].value = M;
            RG[i].free = false;
        }

        if ((RG[i].pos - qf.get_pos()).norm() < mindist && RG[i].free)
        {
            mindist = (RG[i].pos - qf.get_pos()).norm();
            qf_i = i;
        }
    }
    //qf = 0. Minimo global
    RG[qf_i].value = 0;
    L.splice(L.begin(), RG.get_4neighbors(qf_i));

    uint32_t U = 1;
    std::list<CellGrid> next_L;
    while (L.empty() == false)
    {
        for (auto cell : L)
        {
            if (cell.value == -M)
            {
                RG[cell.i].value = U;
                next_L.splice(next_L.begin(), RG.get_4neighbors(cell.i));
            }
        }
        U++;
        L.swap(next_L);
        next_L.clear();
    }

    return RG;
}


bool compare_cell_value(const CellGrid& first, const CellGrid& second)
{
  return ( first.value > second.value );
}

std::list<CellGrid> depthFirst(const RegularGrid &grid,
                               const Config &qi,
                               const Config &qf)
{
    std::list<CellGrid> path, tmp;
    std::list<CellGrid> neighbors;
    std::stack<std::list<CellGrid>> open;
    bool sucess = false;
    bool visited[grid.size()];
    memset(visited, 0, grid.size());


    CellGrid cell_qi = grid.get_cell(qi);
    CellGrid cell_qf = grid.get_cell(qf);
    
    neighbors = grid.get_4neighbors(cell_qi.i);
    //ordena do maior para o menor, pq a pilha vai inverter isso e quero que ela comece a pegar do menor para o maior
    neighbors.sort(compare_cell_value);

    visited[cell_qi.i] = true;
    for(auto q : neighbors)
    {
        visited[q.i] = true;
        open.push( {cell_qi, q} );
    }

    while ((open.empty() == false) && !sucess)
    {
        path = open.top();
        open.pop();
        if(path.front().i == cell_qf.i)
        {
            sucess = true;
            break;
        }

        neighbors = grid.get_4neighbors(path.front().i);
        //ordena do maior para o menor, pq a pilha vai inverter isso e quero que ela comece a pegar do menor para o maior
        neighbors.sort(compare_cell_value);

        for(auto q : neighbors)
        {
            if(visited[q.i] == false)
            {
                tmp = path;
                tmp.push_back(q);
                open.push( tmp );
                visited[q.i] = true;
            }
        }    

    }

    if(sucess == false)
    {
        std::cerr << "Falha ao tentar encontrar caminho por busca em profundidade!\n";
        path.clear();
    }

    return path;
}
#pragma once
#include <configSpaceTools.hpp> //config, Vector2D
#include <occupating_grid.hpp>  //OccupationGrid
#include <vector>
#include <list>

class CellGrid
{
public:
    Vector2D pos;
    uint32_t value;
    bool free;
};

class RegularGrid
{
public:
    RegularGrid(const Vector2D &center,
                const float &width,
                const float &height,
                const float &size_cell = 0.1);
    inline std::vector<CellGrid> get_grid() const { return _regularGrid; }
    inline CellGrid& operator[](const uint32_t &i) { return _regularGrid[i]; }
    inline std::size_t size()const{ return _regularGrid.size(); }

    inline CellGrid& operator()(const uint32_t& row, const uint32_t& col) { return _regularGrid[col + row*_width]; }

private:
    std::vector<CellGrid> _regularGrid;
    float _width;
    float _height;
    float _size_cell;
};

// RegularGrid OccupationGrid2RegularGrid(const OccupationGrid &OG, const Robot &robot);
RegularGrid manhattan(const OccupationGrid &OG,
                      const Config &qf,
                      const Robot &robot,
                      const double &threshold = 0.4);
std::list<Config> depthFirst(const RegularGrid &grid,
                             const Config &qi,
                             const Config &qf);
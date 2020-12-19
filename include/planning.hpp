#pragma once
#include <configSpaceTools.hpp> //config, Vector2D
#include <occupating_grid.hpp>  //OccupationGrid
#include <limits>               // std::numeric_limits
#include <vector>
#include <list>

const int32_t M = std::numeric_limits<int32_t>::max();

class CellGrid
{
public:
    Vector2D pos;
    int32_t value;
    bool free;
    uint32_t i; //index
};

class RegularGrid
{
public:
    RegularGrid(const Vector2D &center,
                const float &width,
                const float &height,
                const float &size_cell = 0.1);
    inline std::vector<CellGrid> get_grid() const { return _regularGrid; }
    inline CellGrid &operator[](const uint32_t &i) { return _regularGrid[i]; }
    inline CellGrid operator[](const uint32_t &i) const { return _regularGrid[i]; }
    inline CellGrid &operator()(const uint32_t &row, const uint32_t &col) { return _regularGrid[col + row * _width]; }

    inline float width() const { return _width; }
    inline float height() const { return _height; }
    inline float get_size_cell() const { return _size_cell; }
    inline std::size_t size() const { return _regularGrid.size(); }

    std::list<CellGrid> get_4neighbors(const uint32_t &i) const;

    uint32_t get_index(const Config &q) const;
    CellGrid get_cell(const Config &q) const { return _regularGrid[get_index(q)]; }

private:
    std::vector<CellGrid> _regularGrid;
    uint16_t _width;
    uint16_t _height;
    float _size_cell;
};

// RegularGrid OccupationGrid2RegularGrid(const OccupationGrid &OG, const Robot &robot);
RegularGrid manhattan(const OccupationGrid &OG,
                      const Config &qf,
                      const Robot &robot,
                      const double &threshold = 0.4);
std::list<CellGrid> depthFirst(const RegularGrid &grid,
                               const Config &qi,
                               const Config &qf);

std::list<CellGrid> bestFirst(RegularGrid &grid,
                              const Config &qi,
                              const Config &qf);
#include <matplotlibcpp.h>
#include <configSpaceTools.hpp>
#include <iostream>  //cout
#include <stdio.h> //printf

#include <list>
#include <vector>
#include <cmath>
#include <string>

using namespace std;
namespace plt = matplotlibcpp;

void plotPolygon(const Polygon2D &polygon, std::string name, int fig = 0, bool show = true);
void plotPolygons(const std::list<Polygon2D> &polygon_list, std::string prefix, int fig = 0, bool show = true);
void polygon_to_vectorsXY(const Polygon2D &polygon, std::vector<double> &vertices_x, std::vector<double> &vertices_y);

int main(int argc, char** argv)
{   
    std::list<Polygon2D> obstacles;
    Polygon2D polygon;
    Robot robot;

    robot.my_shape = Polygon2D::circle_to_polygon2D(1.0, 32);
    Polygon2D b0 = Polygon2D::rectangle_to_polygon2D(4.0, 4.0);
    b0 = b0.translate(Vector2D(-6.0,0.0));

    Polygon2D cb0 = b0.work_to_config_space(robot);
    
    plotPolygon(b0, "B0", 1, false);
    plotPolygon(robot.my_shape, "Robot", 1, false);
    
    plotPolygon(cb0, "CB0", 2, false);
    Polygon2D point;
    point.add_vertex(Vector2D());
    plotPolygon(point, "Robot", 2, true);


    /* Calculo de distancia entre poligonos está falhando ainda
    polygon = Polygon2D::rectangle_to_polygon2D(2.0, 2.0);
    polygon = polygon.translate(Vector2D(-6.0,0.0));
    obstacles.push_back( polygon );

    polygon = Polygon2D::rectangle_to_polygon2D(2.0, 2.0);
    polygon = polygon.rotation(M_PI/4.0);
    polygon = polygon.translate(Vector2D(6.0,0.0));
    obstacles.push_back( polygon );

    cout << "Distancia entre B0 e B1:" << obstacles.front().distance(obstacles.back()) << '\n';

    plotPolygons(obstacles, "B", true);
    */

    /*
    polygon = Polygon2D::circle_to_polygon2D(2.5 , 8).rotation(M_PI/4.0);
    polygon = polygon.translate(Vector2D(4.0, 2.0));
    obstacles.push_back( polygon );

    polygon = Polygon2D::rectangle_to_polygon2D(2.5, 1.0).rotation(M_PI/4.0);
    polygon = polygon.translate(Vector2D(-2.0, -2.0));
    obstacles.push_back(  polygon );


    polygon = Polygon2D::rectangle_to_polygon2D(3.0, 2.0).rotation(-M_PI/4.0);
    polygon = polygon.translate(Vector2D(2.0, -4.0));
    obstacles.push_back( polygon );


    polygon = Polygon2D::rectangle_to_polygon2D(2.0, 5.0);
    polygon = polygon.translate(Vector2D(-4.0, 4.0));
    obstacles.push_back(  polygon );


    plotPolygons(obstacles, "B", false);
    robot.my_shape = Polygon2D::circle_to_polygon2D(0.5, 8);


    plotPolygon(robot.my_shape, "Robot", true);
    */

    // polygon = obstacles.front();
    // polygon = polygon.work_to_config_space(robot);
    // plotPolygon(polygon, "CB", true);

    return 0;
}






void plotPolygons(const std::list<Polygon2D> &polygon_list, std::string prefix, int fig, bool show)
{
    vector<double> vertices_x;
    vector<double> vertices_y;
    int i = 0;
    for(auto polygon_it = polygon_list.begin(); polygon_it != polygon_list.end(); polygon_it++)
    {   
        polygon_to_vectorsXY(*polygon_it, vertices_x, vertices_y);
        plt::figure(fig);
        plt::named_plot(prefix+to_string(i),vertices_x, vertices_y, "-");
        plt::annotate(prefix+to_string(i), vertices_x.back(), vertices_y.back());
        vertices_x.clear();
        vertices_y.clear();
        i++;
    }
    plt::axis("equal");
    if(show)
        plt::show();
}

void plotPolygon(const Polygon2D &polygon, std::string name, int fig, bool show)
{
    vector<double> vertices_x;
    vector<double> vertices_y;
    polygon_to_vectorsXY(polygon, vertices_x, vertices_y);

    plt::figure(fig);
    plt::named_plot(name, vertices_x, vertices_y, "-");
    plt::axis("equal");
    plt::annotate(name, vertices_x.back(), vertices_y.back());

    if(show)
        plt::show();
}

void polygon_to_vectorsXY(const Polygon2D &polygon, std::vector<double> &vertices_x, std::vector<double> &vertices_y)
{
    auto vertices = polygon.get_vertices();

    for(auto it = vertices.begin(); it != vertices.end(); it ++)
    {
        vertices_x.push_back(it->x());
        vertices_y.push_back(it->y());
    }
    vertices_x.push_back( vertices.front().x() );
    vertices_y.push_back( vertices.front().y() );
}
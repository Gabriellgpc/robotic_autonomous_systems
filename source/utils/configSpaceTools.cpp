#include "configSpaceTools.hpp"
#include <cmath>
#include <tuple>     //tuple
#include <algorithm> //find

// Vector2D

// Vector2D

// Polygon2D
typedef std::list<Vector2D>::const_iterator vertex_it;

std::list<Vector2D> Polygon2D::get_vertices() const
{
    return this->my_vertices;
}

std::list<Vector2D> Polygon2D::get_normalVectors() const
{
    std::list<Vector2D> normals;
    vertex_it it = this->my_vertices.begin();
    Vector2D p[2]; //point/vertex p_k and p_k+1
    Vector2D normal_tmp;

    if (Polygon2D::check_convexity(*this) == false)
        return normals;

    while (it != this->my_vertices.end())
    {
        p[0] = *it;
        ++it;
        if (it == this->my_vertices.end())
            p[1] = *this->my_vertices.begin();
        else
            p[1] = *it;

        //vetor paralelo ao lado ai ai+1
        normal_tmp = Vector2D(p[1]) - Vector2D(p[0]);
        //vetor normal ao lado ai ai+1
        normal_tmp = Vector2D(normal_tmp.y(), -normal_tmp.x());
        //normalizando o vetor
        normal_tmp.normalize();
        //adicionado a lista de normais
        normals.push_back(normal_tmp);
    }
    return normals;
}

bool operator<(std::tuple<Vector2D, Vector2D, bool> A, std::tuple<Vector2D, Vector2D, int> B)
{
    return (std::get<1>(A)).ang() < (std::get<1>(B)).ang();
}

Polygon2D Polygon2D::work_to_config_space(const Robot &robot) const
{
    Polygon2D polygon;
    std::list<Vector2D> ob_normals;
    std::list<Vector2D> robot_normals;
    std::list<Vector2D> robot_vertices;
    //tupla contando os vetores normais e os respectivos vertices
    std::list<std::tuple<Vector2D, Vector2D, bool>> normal_vertex_list;

    ob_normals = this->get_normalVectors();
    robot_normals = robot.my_shape.get_normalVectors();
    robot_vertices = robot.my_shape.get_vertices();

    //preenchendo a tupla normal_vertex_list

    //dados relativos ao objeto/obstaculo (indicado pelo numero/flag 0/false)
    std::list<Vector2D>::const_iterator it_normal = ob_normals.begin();
    std::list<Vector2D>::const_iterator it_vertex = this->my_vertices.begin();
    for (int i = 0; i < ob_normals.size(); i++)
    {
        auto normal_vertex = std::make_tuple(*(it_vertex++), *(it_normal++), false);
        normal_vertex_list.push_back(normal_vertex);
    }

    //dados relativos ao robo (indicado pelo numero/flag 1)
    it_normal = robot_normals.begin();
    it_vertex = robot_vertices.begin();
    for (int i = 0; i < robot_normals.size(); i++)
    {
        //empacotando o vertice com a respectiva normal negativada (vertex, -normal, 1/true(flag::robot))
        auto normal_vertex = std::make_tuple(*(it_vertex++), -*(it_normal++), true);
        normal_vertex_list.push_back(normal_vertex);
    }
    //sorting normal_vertex list according to normal vector
    normal_vertex_list.sort();
    //percorrer a lista ordenada e calcular os vertices do C-obstaculo
    auto it_curr = normal_vertex_list.begin();
    //a: vertice do robo no referencial do robo
    //b: vertice do objeto no referencial do mundo
    Vector2D a, b;
    while (it_curr != normal_vertex_list.end())
    {
        bool is_robot = std::get<2>(*it_curr);
        auto it_next = std::find(it_curr, normal_vertex_list.end(), !is_robot);

        if (is_robot)
        {
            a = robot.my_config.my_pos - std::get<0>(*it_curr);
            b = std::get<0>(*it_next);
        }
        else
        {
            a = robot.my_config.my_pos - std::get<0>(*it_next);
            b = std::get<0>(*it_curr);
        }
        polygon.add_vertex(b-a);
    }

    return polygon;
}

bool Polygon2D::check_convexity(const Polygon2D &poly)
{
    if (poly.my_vertices.size() < 3)
        return false;

    vertex_it v_it = poly.my_vertices.begin();
    double a, b, c, d;
    Vector2D pk[2];

    while (v_it != poly.my_vertices.end())
    {
        pk[0] = *v_it;
        v_it++;
        if (v_it == poly.my_vertices.end())
            pk[1] = *poly.my_vertices.begin();
        else
            pk[1] = *v_it;
        //reta que vai de p0 ate p1
        a = pk[0].y() - pk[1].y();
        b = pk[1].x() - pk[0].x();
        c = pk[0].x() * pk[1].y() - pk[1].x() * pk[0].x();
        //verifica se algum dos my_vertices encontra-se no semi plano direito dessa reta (d < 0)
        //Obs.: poligono com my_vertices numerados de forma ordenada no sentido anti-horário
        for (vertex_it it = poly.my_vertices.begin(); it != poly.my_vertices.end(); it++)
        {
            Vector2D p(*it);
            d = (a * p.x() + b * p.y() + c) / sqrt(a * a + b * b);
            if (d < 0.0)
                return false;
        }
    }
    return true;
}
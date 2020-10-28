#include "configSpaceTools.hpp"
#include <cmath>
#include <tuple>     //tuple
#include <algorithm> //find
#include <limits>       // std::numeric_limits

Config::Config() : my_pos(),
                   my_theta(0.0)
{
}
Robot::Robot() : my_config(),
                 my_shape()
{
}
/*********************************************************** Vector2D ***********************************************************/

Vector2D::Vector2D() : _x(0.0),
                       _y(0.0)
{
}
Vector2D::Vector2D(const double x, const double y) : _x(x),
                                                     _y(y)
{
}
Vector2D::Vector2D(const Vector2D &other)
{
    this->_x = other._x;
    this->_y = other._y;
}

Vector2D Vector2D::operator-() const
{
    return Vector2D(-this->_x, -this->_y);
}
Vector2D Vector2D::operator-(const Vector2D &other) const
{
    return Vector2D(this->_x - other._x, this->_y - other._y);
}
Vector2D Vector2D::operator+(const Vector2D &other) const
{
    return Vector2D(this->_x + other._x, this->_y + other._y);
}
Vector2D Vector2D::operator*(const double &k) const
{
    return Vector2D(this->_x * k, this->_y * k);
}
bool Vector2D::operator==(const Vector2D &other) const
{
    const double delta_diff = 1.0e-6; //ordem de 1 micrometro
    Vector2D diff = *this - other;
    return (fabs(diff._x) <= delta_diff) && (fabs(diff._y) <= delta_diff);
}

double Vector2D::operator()(unsigned int i) const
{
    return (i == 0) ? _x : _y;
}
double Vector2D::operator[](unsigned int i) const
{
    return (i == 0) ? _x : _y;
}
double &Vector2D::operator[](unsigned int i)
{
    return (i == 0) ? _x : _y;
}
double Vector2D::norm() const
{
    return sqrt(_x * _x + _y * _y);
}
double Vector2D::dot(const Vector2D &other) const
{
    return this->_x * other._x + this->_y * other._y;
}
//retorna o angulo (theta) do vetor com relação ao eixo x positivo. theta in [0, 2pi]
double Vector2D::ang() const
{
    return atan2(this->_y, this->_x);
}
double Vector2D::ang(const Vector2D &other) const
{
    return atan2(other._y - this->_y, other._x - this->_x);
}
void Vector2D::operator=(const Vector2D &other)
{
    this->_x = other._x;
    this->_y = other._y;
}
void Vector2D::normalize()
{
    double norm = this->norm();
    if (norm == 0.0)
        return;
    this->_x /= norm;
    this->_y /= norm;
}
void Vector2D::translate(const Vector2D &trans)
{
    this->_x += trans._x;
    this->_y += trans._y;
}
void Vector2D::rotation(const double &theta)
{
    this->_x = this->_x * cos(theta) - this->_y * sin(theta);
    this->_y = this->_x * sin(theta) + this->_y * cos(theta);
}

Vector2D Vector2D::translate(const Vector2D &trans) const
{
    Vector2D result;
    result._x = this->_x + trans._x;
    result._y = this->_y + trans._y;
    return result;
}
Vector2D Vector2D::rotation(const double &theta) const
{
    Vector2D result;
    result._x = this->_x * cos(theta) - this->_y * sin(theta);
    result._y = this->_x * sin(theta) + this->_y * cos(theta);
    return result;
}
/*********************************************************** Polygon2D ***********************************************************/
typedef std::list<Vector2D>::const_iterator vertex_it;
bool operator<(std::tuple<Vector2D, Vector2D, bool> A, std::tuple<Vector2D, Vector2D, int> B)
{
    return (std::get<1>(A)).ang() < (std::get<1>(B)).ang();
}

bool operator==(std::tuple<Vector2D, Vector2D, bool> A, bool flag)
{
    return std::get<2>(A) == flag;
}

Polygon2D::Polygon2D() : my_vertices()
{
}
Polygon2D::~Polygon2D()
{
    my_vertices.clear();
}

Polygon2D::Polygon2D(std::list<Vector2D> vertices)
{
    this->my_vertices = vertices;
}
void Polygon2D::add_vertex(const Vector2D vertex)
{
    //verifica se o vertice ja existe
    // find utilizara Vector2D == Vector2D (operator==)
    auto at = std::find(my_vertices.begin(), my_vertices.end(), vertex);
    if (at != my_vertices.end()) //caso o vertice ja exista no poligono
        return;
    my_vertices.push_back(vertex);
}
//retorna o poligono resultante da translação do atual poligono
Polygon2D Polygon2D::translate(const Vector2D &trans) const
{
    Polygon2D result(*this);
    //transladar todos os vertices do poligono
    auto ver_it = result.my_vertices.begin();
    for (ver_it; ver_it != result.my_vertices.end(); ver_it++)
    {
        ver_it->translate(trans);
    }
}
//retorna o poligono resultante da rotação do atual poligono em theta radianos
Polygon2D Polygon2D::rotation(const double &theta) const
{
    Polygon2D result(*this);
    //transladar todos os vertices do poligono
    auto ver_it = result.my_vertices.begin();
    for (ver_it; ver_it != result.my_vertices.end(); ver_it++)
    {
        ver_it->rotation(theta);
    }
}
double Polygon2D::penetration_test(const Vector2D &p) const
{
    vertex_it v_it = this->my_vertices.begin();
    double a, b, c, d;
    double min = std::numeric_limits<double>::max();
    Vector2D pk[2];

    while (v_it != this->my_vertices.end())
    {
        pk[0] = *v_it;
        ++v_it;
        if (v_it == this->my_vertices.end())
            pk[1] = *this->my_vertices.begin();
        else
            pk[1] = *v_it;
        //reta que vai de p0 ate p1
        a = pk[0].y() - pk[1].y();
        b = pk[1].x() - pk[0].x();
        c = pk[0].x() * pk[1].y() - pk[1].x() * pk[0].x();
        
        d = (a * p.x() + b * p.y() + c) / sqrt(a * a + b * b);
        if(d < min)
            min = d;
    }
    return min;
}
double Polygon2D::distance(const Polygon2D &polygon)const
{
    double dist          = std::numeric_limits<double>::max();
    double d;
    Vector2D ai,bj, Vai, Vbj;
    
    auto my_vertex_it    = my_vertices.begin();
    auto my_normals      = this->get_normalVectors();
    auto my_normals_it   = my_normals.begin();
    
    auto poly_vertices   = polygon.get_vertices();
    auto poly_vertex_it  = poly_vertices.begin();
    auto poly_normals    = polygon.get_normalVectors();
    auto poly_normals_it = poly_normals.begin(); 
    
    //percorre todos os vertices deste poligono (this)
    for(my_vertex_it; my_vertex_it != my_vertices.end(); my_vertex_it++)
    {
        ai = *my_vertex_it;
        Vai= *my_normals_it;
        //percorre todos os vertices do outro poligono (polygon)
        for(poly_vertex_it; poly_vertex_it != poly_vertices.end(); poly_vertex_it++)
        {   
            bj = *poly_vertex_it;
            Vbj= *poly_normals_it;

            // contato tipo A
            d = fabs(Vai.dot( bj - ai ));
            dist = (d < dist)?d:dist;
            // contato tipo B
            d = fabs(Vbj.dot( ai - bj ));
            dist = (d < dist)?d:dist;

            poly_normals_it++;
        }
        my_normals_it++;
    }
    return dist;
}
//retorna true caso tenha sobreposição entre os poligonos (entre this e other)
bool Polygon2D::check_overlay(const Polygon2D other) const
{
    // verifica se algum vertice de A esta dentro de B e vice-versa
    
}
void Polygon2D::operator=(const Polygon2D &other)
{
    this->my_vertices = other.my_vertices;
}
Polygon2D Polygon2D::circle_to_polygon2D(const double &radius)
{

}
//p1: vertice inferior esquerdo
//p2: vertice superior direito
Polygon2D Polygon2D::rectangle_to_polygon2D(const Vector2D &p1, const Vector2D &p2)
{

}
Polygon2D Polygon2D::rectangle_to_polygon2D(const Vector2D &center, const double &width, const double height)
{

}
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
    auto curr_normal_it = normal_vertex_list.begin();
    std::list<std::tuple<Vector2D, Vector2D, bool>>::iterator next_normal_it;
    //a: vertice do robo no referencial do robo
    //b: vertice do objeto no referencial do mundo
    Vector2D a, b;
    bool is_robot;
    while (curr_normal_it != normal_vertex_list.end())
    {
        is_robot = std::get<2>(*curr_normal_it);
        /* procura pela proxima normal
        **caso o atual seja a normal do obstaculo (is_robot == false): procurar pela normal seguinte do robo
        **caso o atual seja a normal do robo (is_robot == true): procurar pela normal seguinte do obstaculo
        */
        //TODO: tratar quando for ultimo elemento
        next_normal_it = std::find(curr_normal_it, normal_vertex_list.end(), !is_robot);

        if (is_robot)
        {
            a = robot.my_config.my_pos - std::get<0>(*curr_normal_it);
            b = std::get<0>(*next_normal_it);
        }
        else
        {
            a = robot.my_config.my_pos - std::get<0>(*next_normal_it);
            b = std::get<0>(*curr_normal_it);
        }
        polygon.add_vertex(b - a);
        ++curr_normal_it;
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

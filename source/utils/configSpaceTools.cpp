#include "configSpaceTools.hpp"
#include <cmath>
#include <tuple>     //tuple
#include <algorithm> //find
#include <limits>    // std::numeric_limits]
#include <fstream>   //ifstream, ofstream
#include <iostream>  //std::ostream

Config::Config() : my_pos(),
                   my_theta(0.0)
{
}
Config::Config(const double &x, const double &y, const double &theta) : my_pos(x, y),
                                                                        my_theta(theta)
{
}
Config::Config(const Vector2D &pos, const double &theta) : my_pos(pos),
                                                           my_theta(theta)
{
}
void Config::set_pos(const double &x, const double &y)
{
    my_pos = Vector2D(x, y);
}
void Config::set_pos(const Vector2D &pos)
{
    my_pos = pos;
}
void Config::set_theta(const double &theta)
{
    my_theta = theta;
}
void Config::translate(const Vector2D &t)
{
    my_pos = my_pos + t;
}
void Config::rotate(const double &phi)
{
    my_theta += phi;
    if (my_theta >= 2.0 * M_PI)
        my_theta -= 2.0 * M_PI;
}
Vector2D Config::get_pos() const
{
    return my_pos;
}
double Config::get_theta() const
{
    double th = my_theta;
    // making sure that theta in [0, 2pi]
    if (th >= 2.0 * M_PI)
        th -= 2.0 * M_PI;

    if (th < 0.0)
        th += 2.0 * M_PI;

    return th;
}

Robot::Robot() : my_config(),
                 my_shape()
{
}
Robot::Robot(const Robot &robot)
{
    my_config = robot.my_config;
    my_shape = robot.my_shape;
}
Robot::Robot(const double &x, const double &y, const double &theta, const Polygon2D &shape) : my_config(x, y, theta),
                                                                                              my_shape(shape)
{
}
Robot::Robot(const Config &config, const Polygon2D &shape) : my_config(config),
                                                             my_shape(shape)
{
}
Robot::Robot(const Vector2D &pos, const double &theta, const Polygon2D &shape) : my_config(pos, theta),
                                                                                 my_shape(shape)
{
}
void Robot::set_config(const Config &config)
{
    my_config = config;
}

void Robot::translate(const Vector2D &t)
{
    my_config.translate(t);
}
void Robot::rotate(const double &phi)
{
    my_config.rotate(phi);
}

Polygon2D Robot::to_polygon2D() const
{
    Polygon2D polygon;
    polygon = my_shape.rotation(my_config.get_theta());
    polygon = polygon.translate(my_config.get_pos());
    return polygon;
}
Config Robot::get_config() const
{
    return my_config;
}

Polygon2D Robot::get_shape() const
{
    return my_shape;
}

bool Robot::set_shape(const Polygon2D &shape)
{
    if (Polygon2D::check_convexity(shape) == false)
        return false;

    my_shape = shape;

    return true;
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
    double ang = atan2(this->_y, this->_x); //[-pi, pi]
    if (fabs(ang) < 1.0e-9)
        return 0.0;
    if (ang < 0.0)
        ang += 2 * M_PI;
    return ang;
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
Vector2D Vector2D::normalize() const
{
    Vector2D v_normalized;

    double norm = this->norm();
    if (norm == 0.0)
        return v_normalized;
    v_normalized._x = this->_x / norm;
    v_normalized._y = this->_y / norm;

    v_normalized._x = (fabs(v_normalized._x) < 1.0e-9) ? 0.0 : v_normalized._x;
    v_normalized._y = (fabs(v_normalized._y) < 1.0e-9) ? 0.0 : v_normalized._y;

    return v_normalized;
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
bool operator<(std::tuple<Vector2D, Vector2D, bool> A, std::tuple<Vector2D, Vector2D, bool> B)
{
    double angA = std::get<1>(A).ang();
    double angB = std::get<1>(B).ang();
    return angA < angB;
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
        *ver_it = ver_it->translate(trans);
    }
    return result;
}
Polygon2D Polygon2D::translate(const double &x, const double &y) const
{
    return this->translate(Vector2D(x, y));
}
//retorna o poligono resultante da rotação do atual poligono em theta radianos
Polygon2D Polygon2D::rotation(const double &theta) const
{
    Polygon2D result(*this);
    //transladar todos os vertices do poligono
    auto ver_it = result.my_vertices.begin();
    for (ver_it; ver_it != result.my_vertices.end(); ver_it++)
    {
        *ver_it = ver_it->rotation(theta);
    }
    return result;
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
        //reta que vai de pk ate pk+1
        a = pk[0].y() - pk[1].y();
        b = pk[1].x() - pk[0].x();
        c = pk[0].x() * pk[1].y() - pk[1].x() * pk[0].y();

        d = (a * p.x() + b * p.y() + c) / sqrt(a * a + b * b);
        d = (fabs(d) < 1.0e-6) ? 0.0 : d;
        if (d < min)
            min = d;
    }
    return min;
}
// WARNING: Calculo de distancia entre poligonos está falhando ainda
double Polygon2D::distance(const Polygon2D &polygon) const
{
    double dist = std::numeric_limits<double>::max();
    double d;
    Vector2D ai, bj, Vai, Vbj;

    auto my_vertex_it = my_vertices.begin();
    auto my_normals = this->get_normalVectors();
    auto my_normals_it = my_normals.begin();

    auto poly_vertices = polygon.get_vertices();
    auto poly_vertex_it = poly_vertices.begin();
    auto poly_normals = polygon.get_normalVectors();
    auto poly_normals_it = poly_normals.begin();

    //percorre todos os vertices deste poligono (this/A)
    int ia = 0, jb = 0;
    for (my_vertex_it; my_vertex_it != my_vertices.end(); my_vertex_it++)
    {
        ai = *my_vertex_it;
        Vai = *my_normals_it;
        //percorre todos os vertices do outro poligono (polygon/B)
        poly_normals_it = poly_normals.begin();

        jb = 0;
        for (poly_vertex_it = poly_vertices.begin(); poly_vertex_it != poly_vertices.end(); poly_vertex_it++)
        {
            bj = *poly_vertex_it;
            Vbj = *poly_normals_it;

            // contato tipo A
            d = Vai.dot(bj - ai);
            printf("\nai:%d -> bj:%d\n", ia, jb);
            std::cout << "d:" << d << '\n';

            d = (d <= 0.0) ? std::numeric_limits<double>::max() : d;
            dist = (d < dist) ? d : dist;

            // contato tipo B
            d = Vbj.dot(ai - bj);
            printf("bj:%d -> ai:%d\n", jb, ia);
            std::cout << "d:" << d << '\n';

            d = (d <= 0.0) ? std::numeric_limits<double>::max() : d;
            dist = (d < dist) ? d : dist;

            poly_normals_it++;
            jb++;
        }
        my_normals_it++;
        ia++;
    }
    return dist;
}
//retorna true caso tenha sobreposição entre os poligonos (entre this e other)
bool Polygon2D::check_overlay(const Polygon2D &other) const
{
    // verifica se algum vertice de A esta dentro de B e vice-versa
    auto other_vertices = other.get_vertices();
    auto other_vertices_it = other_vertices.begin();
    auto my_vertices_it = this->my_vertices.begin();
    double d;

    //testa se algum vertice de Other esta dentro deste poligono (this)
    for (other_vertices_it; other_vertices_it != other_vertices.end(); other_vertices_it++)
    {
        d = this->penetration_test(*other_vertices_it);
        //caso bj esteja dentro de A, o teste de penetração resultara em um número positivo
        if (d > 0.0)
            return true;
    }

    if (Polygon2D::check_convexity(other) == false)
        return false;
    //testa se algum vertice deste poligono (this) esta dentro de Other (apenas dse other for convexo)
    for (my_vertices_it; my_vertices_it != my_vertices.end(); my_vertices_it++)
    {
        d = other.penetration_test(*my_vertices_it);
        //caso ai esteja dentro de B, o teste de penetração resultara em um número positivo
        if (d > 0.0)
            return true;
    }

    return false;
}
void Polygon2D::operator=(const Polygon2D &other)
{
    this->my_vertices = other.my_vertices;
}

Polygon2D Polygon2D::circle_to_polygon2D(const double &radius, const unsigned int num_vertices)
{
    return Polygon2D::circle_to_polygon2D(Vector2D(0, 0), radius, num_vertices);
}
Polygon2D Polygon2D::circle_to_polygon2D(const Vector2D &center, const double &radius, const unsigned int num_vertices)
{
    Polygon2D polygon;
    Vector2D p0, vertex_tmp;
    double phi = M_PI / num_vertices;
    p0 = Vector2D(radius, radius * tan(phi));

    for (unsigned int i = 0; i < num_vertices; i++)
    {
        vertex_tmp = p0.rotation(2.0 * i * phi);
        vertex_tmp = vertex_tmp + center;
        polygon.add_vertex(vertex_tmp);
    }
    return polygon;
}
//p1: vertice inferior esquerdo
//p2: vertice superior direito
Polygon2D Polygon2D::rectangle_to_polygon2D(const Vector2D &p1, const Vector2D &p2)
{
    Polygon2D polygon;
    Vector2D center, p1p2 = p2 - p1;
    double width, height, norm = p1p2.norm(), ang = p1p2.ang();

    center = (p2 + p1) * 0.5; //(p1+p2)/2.0
    width = norm * cos(ang);
    height = norm * sin(ang);

    return Polygon2D::rectangle_to_polygon2D(center, width, height);
}
Polygon2D Polygon2D::rectangle_to_polygon2D(const Vector2D &center, const double &width, const double height)
{
    Polygon2D polygon;
    Vector2D p_tmp;

    //vertice 1
    p_tmp = center + Vector2D(-width / 2.0, -height / 2.0);
    polygon.add_vertex(p_tmp);

    //vertice 2
    p_tmp = center + Vector2D(width / 2.0, -height / 2.0);
    polygon.add_vertex(p_tmp);

    //vertice 3
    p_tmp = center + Vector2D(width / 2.0, height / 2.0);
    polygon.add_vertex(p_tmp);

    //vertice 4
    p_tmp = center + Vector2D(-width / 2.0, height / 2.0);
    polygon.add_vertex(p_tmp);

    return polygon;
}

Polygon2D Polygon2D::rectangle_to_polygon2D(const double &width, const double height)
{
    return Polygon2D::rectangle_to_polygon2D(Vector2D(0, 0), width, height);
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
        normal_tmp = normal_tmp.normalize();
        //adicionado a lista de normais
        normals.push_back(normal_tmp);
    }
    return normals;
}

Polygon2D Polygon2D::work_to_config_space(const Polygon2D &robot) const
{
    Polygon2D polygon;
    std::list<Vector2D> ob_normals;
    std::list<Vector2D> robot_normals;
    std::list<Vector2D> robot_vertices;
    //tupla contando os vetores normais e os respectivos vertices
    std::list<std::tuple<Vector2D, Vector2D, bool>> normal_vertex_list;

    ob_normals = this->get_normalVectors();
    robot_normals = robot.get_normalVectors();
    robot_vertices = robot.get_vertices();

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
        auto normal_vertex = std::make_tuple(*(it_vertex++), *(it_normal++) * -1.0, true);
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
        if (next_normal_it == normal_vertex_list.end())
        {
            next_normal_it = std::find(normal_vertex_list.begin(), normal_vertex_list.end(), !is_robot);
        }

        if (is_robot)
        {
            a = std::get<0>(*curr_normal_it);
            b = std::get<0>(*next_normal_it);
        }
        else
        {
            a = std::get<0>(*next_normal_it);
            b = std::get<0>(*curr_normal_it);
        }
        polygon.add_vertex(b - a);
        ++curr_normal_it;
    }

    //calculo para o ultimo ponto/vertice (estou tratando fora do loop pq a lista não é circular)
    auto last = normal_vertex_list.back();
    is_robot = std::get<2>(last);
    next_normal_it = std::find(normal_vertex_list.begin(), normal_vertex_list.end(), !is_robot);
    if (is_robot)
    {
        a = std::get<0>(last);
        b = std::get<0>(*next_normal_it);
    }
    else
    {
        a = std::get<0>(*next_normal_it);
        b = std::get<0>(last);
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
        c = pk[0].x() * pk[1].y() - pk[1].x() * pk[0].y();
        //verifica se algum dos my_vertices encontra-se no semi plano direito dessa reta (d < 0)
        //Obs.: poligono com my_vertices numerados de forma ordenada no sentido anti-horário
        for (vertex_it it = poly.my_vertices.begin(); it != poly.my_vertices.end(); it++)
        {
            Vector2D p(*it);
            d = (a * p.x() + b * p.y() + c) / sqrt(a * a + b * b);
            d = (fabs(d) < 1.0e-6) ? 0.0 : d;
            if (d < 0.0)
                return false;
        }
    }
    return true;
}

void Polygon2D::polygon_to_vectorsXY(const Polygon2D &polygon, std::vector<double> &vertices_x, std::vector<double> &vertices_y)
{
    auto vertices = polygon.get_vertices();

    for (auto it = vertices.begin(); it != vertices.end(); it++)
    {
        vertices_x.push_back(it->x());
        vertices_y.push_back(it->y());
    }

    if (vertices.size() > 1.0)
    {
        vertices_x.push_back(vertices.front().x());
        vertices_y.push_back(vertices.front().y());
    }
}

double Polygon2D::distance(const Vector2D &p) const
{
    auto normals = get_normalVectors();
    auto normal_it = normals.begin();
    double d = 0, d_tmp;

    for (auto v_it = my_vertices.begin(); v_it != my_vertices.end(); v_it++, normal_it++)
    {
        d_tmp = (*normal_it).dot(p - *v_it);
        if (d_tmp < 0.0)
            continue;
        d += d_tmp * d_tmp;
    }

    return sqrt(d);
}

double Polygon2D::area()
{
    double area = 0.0;

    if (Polygon2D::check_convexity(*this) == false)
        return -1.0;

    std::list<Vector2D>::iterator v_next = my_vertices.begin();
    for (auto v_it = my_vertices.begin(); v_it != my_vertices.end(); v_it++)
    {
        ++v_next;
        if (v_next == my_vertices.end())
            v_next = my_vertices.begin();
        area += (v_it->x() * v_next->y()) - (v_next->x() * v_it->y());
    }

    return area * 0.5;
}
Vector2D Polygon2D::center()
{
    Vector2D c(0, 0);

    std::list<Vector2D>::iterator v_next = my_vertices.begin();
    for (auto v_it = my_vertices.begin(); v_it != my_vertices.end(); v_it++)
    {
        ++v_next;
        if (v_next == my_vertices.end())
            v_next = my_vertices.begin();

        c.x() += (v_it->x() + v_next->x()) * ((v_it->x() * v_next->y()) - (v_next->x() * v_it->y()));
        c.y() += (v_it->y() + v_next->y()) * ((v_it->x() * v_next->y()) - (v_next->x() * v_it->y()));
    }

    c.x() *= (1.0 / (6.0 * this->area()));
    c.y() *= (1.0 / (6.0 * this->area()));
    return c;
}

double Polygon2D::min_radius()
{
    Vector2D c, center_to_vertex;
    double radius = -1;
    double d;
    c = this->center();

    for (auto v_it = my_vertices.begin(); v_it != my_vertices.end(); v_it++)
    {
        center_to_vertex = *v_it - c;
        d = center_to_vertex.norm();
        if (d > radius)
            radius = d;
    }

    return radius;
}

void Polygon2D::load_from_istream(std::istream &I)
{
    Polygon2D poly_tmp;
    int num_vertices = 0;
    std::string keyword;
    double x, y;

    std::getline(I, keyword, ':');
    if (keyword != "N")
        return;
    I >> num_vertices;

    for (int i = 0; i < num_vertices; i++)
    {
        I >> x;
        I >> y;
        poly_tmp.add_vertex(Vector2D(x, y));
    }

    *this = poly_tmp;
}
std::ostream &Polygon2D::save_to_ostream(std::ostream &O) const
{
    O << "N:" << this->my_vertices.size() << '\n';
    for (auto it = my_vertices.begin(); it != my_vertices.end(); it++)
    {
        O << it->x() << ' ' << it->y() << '\n';
    }
    return O;
}
bool Polygon2D::save_to_file(const std::string fileName)
{
    std::ofstream outFile(fileName, std::ofstream::out);
    if (outFile.is_open() == false)
        return false;
    outFile << (*this);
    outFile.close();
    return true;
}
bool Polygon2D::load_from_file(const std::string fileName)
{
    std::ifstream input(fileName, std::ifstream::in);

    if (input.is_open() == false)
        return false;

    input >> (*this);

    input.close();
    return true;
}

void operator>>(std::istream &I, Polygon2D &polygon)
{
    polygon.load_from_istream(I);
}
std::ostream &operator<<(std::ostream &O, const Polygon2D &polygon)
{
    polygon.save_to_ostream(O);
    return O;
}

/*********************************************************************************************/
World::World() : my_obstacles(),
                 my_robot(),
                 my_n_samples(100),
                 my_CBs(NULL)
{
}
World::World(std::list<Polygon2D> &obstacles, const Robot &robot) : my_obstacles(obstacles),
                                                                    my_robot(robot),
                                                                    my_n_samples(100),
                                                                    my_CBs(NULL)
{
}
World::World(const Robot &robot) : my_obstacles(),
                                   my_robot(robot),
                                   my_n_samples(100),
                                   my_CBs(NULL)
{
}

World::World(const World &w)
{
    this->operator=(w);
}

World::~World()
{
    this->remove_all_obstacles();
}

void World::operator=(const World &w)
{
    this->my_obstacles = w.my_obstacles;
    this->my_n_samples = w.my_n_samples;
    this->my_robot = w.my_robot;

    if (this->my_CBs != NULL)
        delete[] my_CBs;
    if (w.my_CBs != NULL)
    {
        my_CBs = new std::list<Polygon2D>[this->my_n_samples];
        for (int i = 0; i < this->my_n_samples; i++)
        {
            my_CBs[i] = w.my_CBs[i];
        }
    }
}

void World::add_obstacle(const Polygon2D &obstacle)
{
    my_obstacles.push_back(obstacle);
}
void World::update_config(const Config &config)
{
    my_robot.set_config(config);
}
std::list<Polygon2D> World::get_obstacles() const
{
    return my_obstacles;
}
Robot World::get_robot() const
{
    return my_robot;
}
void World::compute_c_obstacles(const unsigned int n_samples)
{
    double step_angle = 2.0 * M_PI / n_samples;
    Polygon2D robot_th = my_robot.get_shape();

    if ((n_samples == my_n_samples) && (my_CBs != NULL)) //ja computado
        return;

    if (my_CBs != NULL)
    {
        for (int i = 0; i < my_n_samples; i++)
        {
            my_CBs[i].clear();
        }
        delete[] my_CBs;
    }
    my_n_samples = n_samples;
    my_CBs = new std::list<Polygon2D>[my_n_samples];

    for (int i = 0; i < my_n_samples; i++)
    {
        robot_th = robot_th.rotation(step_angle * i);
        for (auto obstacle_it = my_obstacles.begin(); obstacle_it != my_obstacles.end(); obstacle_it++)
        {
            my_CBs[i].push_back(obstacle_it->work_to_config_space(robot_th));
        }
    }
}
std::list<Polygon2D> World::get_cobstacles(const unsigned int n_samples)
{
    unsigned int CB_index;
    double step_angle = 2.0 * M_PI / n_samples;
    double curr_th = my_robot.get_config().get_theta();
    //ira computar apenas se necessario os CBs
    this->compute_c_obstacles(n_samples);

    CB_index = round(curr_th / step_angle);
    CB_index = (CB_index >= n_samples) ? n_samples - 1 : CB_index;

    // printf("Theta = %lf => CB_index = %d\n", curr_th, CB_index);

    return my_CBs[CB_index];
}
void World::remove_all_obstacles()
{
    my_obstacles.clear();
    if (my_CBs == NULL)
    {
        my_n_samples = 0;
        return;
    }

    for (int i = 0; i < my_n_samples; i++)
    {
        my_CBs[i].clear();
    }
    my_n_samples = 0;
    delete[] my_CBs;
}
void World::set_robot(const Robot &robot)
{
    my_robot = robot;
}

bool World::check_collision()
{
    Polygon2D robot_point; //robo em espaco de configuracao
    robot_point.add_vertex(my_robot.get_config().get_pos());
    std::list<Polygon2D> cb_obstacles;

    cb_obstacles = this->get_cobstacles(my_n_samples);

    int ob_id = 0;
    for (auto obstacle_it = cb_obstacles.begin(); obstacle_it != cb_obstacles.end(); obstacle_it++)
    {
        if (obstacle_it->check_overlay(robot_point) == true)
        {
            printf("Ponto (%lf , %lf) esta colidindo com o objetivo:%d\n",
                   my_robot.get_config().get_pos().x(),
                   my_robot.get_config().get_pos().y(),
                   ob_id);
            return true;
        }
        ob_id++;
    }

    return false;
}

// EXTRAS

std::ostream &operator<<(std::ostream &O, const Vector2D &v)
{
    O << '<' << v.x() << ',' << v.y() << '>';
    return O;
}
std::ostream &operator<<(std::ostream &O, const Config &q)
{
    O << "(x:" << q.x() << ", y:" << q.y() << ", th:" << q.theta() << ')';
    return O;
}
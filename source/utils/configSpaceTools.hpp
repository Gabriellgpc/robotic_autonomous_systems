#pragma once
#include <list>
#include <tuple>
#include <iostream>
#include <string>

class Config;
class Robot;

//Todas as unidades de medidas no padrão SI
class Vector2D{
public:
    Vector2D();
    Vector2D(const double x, const double y);
    Vector2D(const Vector2D &other);

    Vector2D operator-()const;
    Vector2D operator-(const Vector2D &other)const;
    Vector2D operator+(const Vector2D &other)const;
    Vector2D operator*(const double &k)const;
    
    bool operator==(const Vector2D &other)const;

    inline double x()const{return _x;}
    inline double y()const{return _y;}
    double operator()(unsigned int i)const;
    double operator[](unsigned int i)const;
    double &operator[](unsigned int i);
    double norm()const;
    double dot(const Vector2D &other)const;
    //retorna o angulo (theta) do vetor com relação ao eixo x positivo. theta in [0, 2pi]
    double ang()const;
    double ang(const Vector2D &other)const;
    Vector2D translate(const Vector2D &trans)const; //[m]
    Vector2D rotation(const double &theta)const;    //[rad]
    Vector2D normalize()const;
    
    void operator=(const Vector2D &other);
    // void normalize();
    // void translate(const Vector2D &trans); //[m]
    // void rotation(const double &theta);   //[rad]
private:
    double _x;
    double _y;
};

class Polygon2D{
public:
    Polygon2D();
    ~Polygon2D();
    
    Polygon2D(std::list<Vector2D> vertices);
    void add_vertex(const Vector2D vertex);

    Polygon2D work_to_config_space(const Polygon2D &robot)const;
    //retorna o poligono resultante da translação do atual poligono
    Polygon2D translate(const Vector2D &trans)const; //[m]
    //retorna o poligono resultante da rotação do atual poligono em theta radianos
    Polygon2D rotation(const double &theta)const;   //[rad]
    
    //retorna um vetor contendo Vector2D para cada normal face/lado do poligono
    std::list<Vector2D> get_normalVectors()const;
    std::list<Vector2D> get_vertices()const;

    double penetration_test(const Vector2D &p)const;
    double distance(const Polygon2D &polygon)const;
    //retorna true caso tenha sobreposição entre os poligonos (entre this e other)
    bool check_overlay(const Polygon2D &other)const;

    //operador de atribuição
    void operator=(const Polygon2D &other);

    static bool check_convexity(const Polygon2D &poly);
    static Polygon2D circle_to_polygon2D(const Vector2D &center, const double &radius, const unsigned int num_vertices = 8);
    static Polygon2D circle_to_polygon2D(const double &radius, const unsigned int num_vertices = 8);
    //p1: vertice inferior esquerdo
    //p2: vertice superior direito
    static Polygon2D rectangle_to_polygon2D(const Vector2D &p1, const Vector2D &p2);
    static Polygon2D rectangle_to_polygon2D(const Vector2D &center, const double &width, const double height);
    static Polygon2D rectangle_to_polygon2D(const double &width, const double height);

    void load_from_istream(std::istream &I);
    std::ostream &save_to_ostream(std::ostream &O)const;
    bool save_to_file(const std::string fileName);
    bool load_from_file(const std::string fileName);
private:
    std::list<Vector2D> my_vertices;
};
void operator>>(std::istream &I, Polygon2D &polygon);
std::ostream &operator<<(std::ostream &O, const Polygon2D &polygon);
bool operator<(std::tuple<Vector2D, Vector2D, bool> A, std::tuple<Vector2D, Vector2D, bool> B);
bool operator==(std::tuple<Vector2D, Vector2D, bool> A, bool flag);

/***************************************************************************************************************/

class Config{
public:
    Config();
    Config(const double &x, const double &y, const double &theta);
    Config(const Vector2D &pos, const double& theta);

    void set_pos(const double &x, const double &y);
    void set_pos(const Vector2D &pos);
    void set_theta(const double &theta);
    void translate(const Vector2D &t);
    void rotate(const double &phi);


    Vector2D get_pos()const;
    double   get_theta()const;
private:
    Vector2D my_pos;
    double  my_theta;
};

class Robot{
public:
    Robot();
    Robot(const Robot &robot);
    Robot(const double &x, const double &y, const double &theta, const Polygon2D &shape);
    Robot(const Config &config, const Polygon2D &shape);
    Robot(const Vector2D &pos, const double &theta, const Polygon2D &shape);

    void set_config(const Config &config);
    void translate(const Vector2D &t);
    void rotate(const double &phi);
    
    Polygon2D to_polygon2D();
    Config get_config()const;
    Polygon2D get_shape()const;
private:
    Polygon2D my_shape;
    Config my_config;
};

/***************************************************************************************************************/

class World{
public:
    World(std::list<Polygon2D> &obstacles, const Robot &robot);
    World(const Robot &robot);
    ~World();

    void add_obstacle(const Polygon2D &obstacle);
    void update_config(const Config &config);

    Robot get_robot()const;
    std::list<Polygon2D> get_obstacles()const;
    std::list<Polygon2D> get_cobstacles(const unsigned int n_samples=100);

    void compute_c_obstacles(const unsigned int n_samples = 100);
    void remove_all_obstacles();
    void set_robot(const Robot &robot);

    bool check_collision();

    void load_world_from_file(const std::string &fileName);
    void save_world_to_file(const std::string fileName)const;
private:
    std::list<Polygon2D> my_obstacles;
    std::list<Polygon2D> *my_CBs;//theta 0 to 2pi
    unsigned int my_n_samples;
    Robot my_robot;
};

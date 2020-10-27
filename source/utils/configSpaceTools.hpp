#pragma once
#include <list>
#include <tuple>
//Todas as unidades de medidas no padrão SI

class Vector2D{
public:
    Vector2D();
    Vector2D(const double _x, const double _y);
    Vector2D(const Vector2D &other);

    Vector2D operator-()const;
    Vector2D operator-(const Vector2D &other)const;
    Vector2D operator+(const Vector2D &other)const;
    Vector2D operator*(const double &k)const;
    
    bool operator==(const Vector2D &other)const;

    double x()const;
    double y()const;
    double norm()const;
    double dot(const Vector2D &other)const;
    //retorna o angulo (theta) do vetor com relação ao eixo x positivo. theta in [0, 2pi]
    double ang()const;
    
    void operator=(const Vector2D &other);
    void normalize();
    void translate(const Vector2D &trans); //[m]
    void rotation(const double &theta);   //[rad]
private:
    double x;
    double y;
};

class Polygon2D{
public:
    Polygon2D();
    ~Polygon2D();
    
    Polygon2D(std::list<Vector2D> _vertices);
    void add_vertex(const Vector2D _vertex);

    Polygon2D work_to_config_space(const Robot &robot)const;
    //retorna o poligono resultante da translação do atual poligono
    Polygon2D translate(const double &x, const double &y)const; //[m]
    Polygon2D translate(const Vector2D &trans)const; //[m]
    //retorna o poligono resultante da rotação do atual poligono em theta radianos
    Polygon2D rotation(const double &theta)const;   //[rad]
    
    //retorna um vetor contendo Vector2D para cada normal face/lado do poligono
    std::list<Vector2D> get_normalVectors()const;
    std::list<Vector2D> get_vertices()const;

    double penetration_test(const Vector2D &p)const;
    //retorna true caso tenha sobreposição entre os poligonos (entre this e other)
    bool check_overlay(const Polygon2D other)const;

    //operador de atribuição
    void operator=(const Polygon2D &other);

    static bool check_convexity(const Polygon2D &poly);
    static Polygon2D circle_to_polygon2D(const double &radius);
    //p1: vertice inferior esquerdo
    //p2: vertice superior direito
    static Polygon2D rectangle_to_polygon2D(const Vector2D &p1, const Vector2D &p2);
    static Polygon2D rectangle_to_polygon2D(const Vector2D &center, const double &width, const double height);
private:
    std::list<Vector2D> my_vertices;
};

class Config{
public:
    Config();

    Vector2D my_pos;
    double  my_theta;
};

class Robot{
public:
    Robot();

    Polygon2D my_shape;
    Config my_config;
};
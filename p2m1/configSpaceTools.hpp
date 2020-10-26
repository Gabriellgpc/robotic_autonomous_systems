#pragma once
#include <list>
#include <vector>
//Todas as unidades de medidas no padrão SI

class Coord2D{
public:
    Coord2D():x(0.0), y(0.0){}
    Coord2D(double _x, double _y):x(_x),y(_y){}

    bool operator==(const Coord2D &other);

    double x;
    double y;
};

class Vector2D{
public:
    Vector2D();
    Vector2D(const double _x, const double _y);
    Vector2D(const Coord2D &point);

    Vector2D operator-(const Vector2D &other)const;
    Vector2D operator+(const Vector2D &other)const;
    bool operator==(const Vector2D &other)const;

    double x()const;
    double y()const;
    double norm()const;
    double dot(const Vector2D &other)const;
    
    void operator=(const Vector2D &other);
    void normalize();
    void translate(const Coord2D &trans); //[m]
    void rotation(const double &theta);   //[rad]
private:
    Coord2D vec;
};

class Polygon2D{
public:
    Polygon2D();
    ~Polygon2D();
    
    Polygon2D(std::list<Coord2D> _vertices);
    void add_vertex(const Coord2D _vertex);
    
    void translate(const double &x, const double &y); //[m]
    void translate(const Coord2D &trans); //[m]
    void rotation(const double &theta);   //[rad]
    
    double penetration_test(const Coord2D &p)const;
    bool check_overlay(const Polygon2D other)const;

    static bool check_convexity(const Polygon2D &poly);
    static Polygon2D circle_to_polygon2D(const double &radius);
    //p1: vertice inferior esquerdo
    //p2: vertice superior direito
    static Polygon2D rectangle_to_polygon2D(const Coord2D &p1, const Coord2D &p2);
    static Polygon2D rectangle_to_polygon2D(const Coord2D &center, const double &width, const double height);
private:
    std::list<Coord2D> vertices;
};
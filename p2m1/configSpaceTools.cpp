#include "configSpaceTools.hpp"
#include <cmath>

typedef std::list<Coord2D>::const_iterator vertex_it;

bool Polygon2D::check_convexity(const Polygon2D &poly)
{
    if(poly.vertices.size() < 3)return false;

    vertex_it v_it = poly.vertices.begin();
    double a,b,c,d;
    Coord2D pk[2];
    
    while(v_it != poly.vertices.end()){
        pk[0] = *v_it;
        v_it++;    
        if(v_it == poly.vertices.end())
            pk[1] = *poly.vertices.begin();
        else
            pk[1] = *v_it;
        //reta que vai de p0 ate p1
        a = pk[0].y - pk[1].y;
        b = pk[1].x - pk[0].x;
        c = pk[0].x*pk[1].y - pk[1].x*pk[0].x;
        //verifica se algum dos vertices encontra-se no semi plano direito dessa reta (d < 0)
        //Obs.: poligono com vertices numerados de forma ordenada no sentido anti-horário
        for(vertex_it it = poly.vertices.begin(); it != poly.vertices.end(); it++){
            Coord2D p(*it);
            d = (a*p.x + b*p.y + c)/sqrt(a*a + b*b);
            if(d < 0.0)return false;
        }    
    }
    return true;
}
#include "utils.hpp"

// #include "b0RemoteApi.h"
// #include "common.hpp"

#include <cmath>

// função que calcula a integral de linha do polinomio de terceiro grau
// aproximação por serie de Taylor
double poly3Length(const double coef[], const double l){
    const double NumPoints = 1000.0; //dlambda
    double integral = 0.0, Dx, Dy;
    double a0,a1,a2,a3,b0,b1,b2,b3;
    a0 = coef[0]; a1 = coef[1]; a2 = coef[2]; a3 = coef[3];
    b0 = coef[4]; b1 = coef[5]; b2 = coef[6]; b3 = coef[7];

    double dlambda = 1.0/NumPoints;
    for(double lambda = 0.0; lambda < l; lambda += dlambda){
        Dx = a1 + 2.0*a2*lambda + 3.0*a3*lambda*lambda;
        Dy = b1 + 2.0*b2*lambda + 3.0*b3*lambda*lambda;
        integral += sqrt(Dx*Dx + Dy*Dy) * dlambda;
    }    

    return integral;
}

double curvature(const double coef[], const double l){
    double Dx = coef[1] + 2*coef[2]*l + 3*coef[3]*l*l;
    double Dy = coef[5] + 2*coef[6]*l + 3*coef[7]*l*l;
    double DDx = 2*coef[2] + 6*coef[3]*l;
    double DDy = 2*coef[6] + 6*coef[7]*l;
    
    double kappa = (DDy*Dx - DDx*Dy)/pow( Dx*Dx + Dy*Dy, 3.0/2.0);
    return fabs(kappa);
}

void poly3(const double coef[], const double l, float &x, float &y, float &th){
    double l2 = l*l;
    double l3 = l2*l;
    
    x = coef[0] + coef[1]*l + coef[2]*l2 + coef[3]*l3;
    y = coef[4] + coef[5]*l + coef[6]*l2 + coef[7]*l3;
    th= atan2f64(coef[5] + 2*coef[6]*l + 3*coef[7]*l2, 
                 coef[1] + 2*coef[2]*l + 3*coef[3]*l2);
}

void pathGenerator(const double coef[],const uint32_t numPoints, float x[], float y[], float th[]){
    double l = 0, step_l = 1.0/numPoints;
    for(int i = 0; i < numPoints; i++)
    {   
        poly3(coef,l,x[i], y[i], th[i]);
        l += step_l;
    }
}

void pathComputing(float xi, float yi, float thi, float xf, float yf, float thf, double coef[]){
    const double delta = 0.001;
    double dx = xf - xi;
    double dy = yf - yi;
    double *a0,*a1,*a2,*a3,*b0,*b1,*b2,*b3;
    a0 = &coef[0]; a1 = &coef[1]; a2 = &coef[2]; a3 = &coef[3];
    b0 = &coef[4]; b1 = &coef[5]; b2 = &coef[6]; b3 = &coef[7];

    bool thi_test = ((M_PI_2 - delta) < thi) && (thi < (M_PI_2 + delta));
    bool thf_test = ((M_PI_2 - delta) < thf) && (thf < (M_PI_2 + delta));

    if(thi_test && thf_test){
        // # caso especial 1
        *b1 = dy;    //#coef. livre
        *b2 = 0;     //#coef. livre
        *a0 = xi;
        *a1 = 0;
        *a2 = 3*dx;
        *a3 = -2*dx;
        *b0 = yi;
        *b3 = dy - (*b1) - (*b2);
    }
    else if(thi_test){
        // #caso especial 2
        double alpha_f = tanf64(thf);
        *a3 = -dx/2.0;  //#coef. livre
        *b3 = 0;        //#coef. livre (qualquer valor aqui)
        *a0 = xi;
        *a1 = 0;
        *a2 = dx - (*a3);
        *b0 = yi;
        *b1 = 2*(dy - alpha_f*dx) - alpha_f*(*a3) + (*b3);
        *b2 = (2*alpha_f*dx - dy) + alpha_f*(*a3) - 2*(*b3);
    }
    else if(thf_test){
        // #caso especial 3
        double alpha_i = tanf64(thi);
        *a1 = 3*dx/2.0;  //#coef. livre
        *b2 = 0;         //#coef. livre (qualquer valor aqui)
        *a0 = xi;
        *a2 = 3*dx - 2*(*a1);
        *a3 = (*a1) - 2*dx;
        *b0 = yi;
        *b1 = alpha_i*(*a1);
        *b3 = dy - alpha_i*(*a1) - (*b2);
    }
    else{
        // #caso geral
        double alpha_i = tanf64(thi);
        double alpha_f = tanf64(thf);
        *a1 = dx;       //#coef. livre
        *a2 = 0;        //#coef. livre
        *a0 = xi;
        *a3 = dx - (*a1) - (*a2);
        *b0 = yi;
        *b1 = alpha_i*(*a1);
        *b2 = 3*(dy - alpha_f*dx) + 2*(alpha_f - alpha_i)*(*a1) + alpha_f*(*a2);
        *b3 = 3*alpha_f*dx - 2*dy - (2*alpha_f - alpha_i)*(*a1) - alpha_f*(*a2);
    }
}

void pioneer_model(float v, float w, float &w_right, float &w_left)
{
    const static float b = 0.331;     //wheel axis distance [m]
    const static float r = 0.09751;   //wheel radius [m]
    w_right = (1.0/r)*v + (b/(2.0*r))*w;
    w_left  = (1.0/r)*v - (b/(2.0*r))*w;
}
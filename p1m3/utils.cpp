#include "utils.hpp"
#include "b0RemoteApi.h"


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

void interPoly3(float xi, float yi, float thi, float xf, float yf, float thf, double coef[]){
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
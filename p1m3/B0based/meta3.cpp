// Enabling the B0-based remote API - client side
// REF.: https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm

#include "b0RemoteApi.h"

#include <cstdio>
#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>

using namespace std;

typedef std::tuple<double, double, double> config_t;

// xi, yi, thi => ponto e orientação inicial
// xf, yf, thf => ponto e orientação final
// coef : [a0,a1,a2,a3,b0,b1,b2,b3]
void interPoly3(const float xi, 
                const float yi, 
                const float thi, 
                const float xf, 
                const float yf, 
                const float thf, 
                double coef[]);
void pathGenerator(const double coef[], 
                   const uint32_t numPoints, 
                   double x[], double y[], double th[]);
void poly3(const double coef[], 
           const double l, 
           double &x, double &y, double &th);

#define N 200

int main(){
    double path_coef[8];
    double x_v[N], y_v[N], th_v[N];
    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    cout << "Conectado!\n";
    client.simxStartSimulation(client.simxServiceCall());    

    interPoly3(0,0,0, 2.5,2.5, M_PI_4f64, path_coef);
    pathGenerator(path_coef, N, x_v, y_v, th_v);
    int lineSize = 1;
    int colorRed[3] = {255,0,0};
    
    // cout << "Some points:\n";
    float segment[6];
    for(int i = 1; i < N; i++){
        // printf("i:%d | (%.2f, %.2f, %.2f)\n", i, x_v[i], y_v[i], th_v[i]);
        segment[0] = x_v[i-1]; // x 
        segment[1] = y_v[i-1]; // y
        segment[2] = 0.3;      // z
        segment[3] = x_v[i]; // x 
        segment[4] = y_v[i]; // y
        segment[5] = 0.3;      // z
        client.simxAddDrawingObject_segments(lineSize, colorRed, segment, 6, client.simxServiceCall());
    }

    client.simxSleep(30*1000);
    client.simxStopSimulation(client.simxServiceCall());

    return 0;
}

void poly3(const double coef[], const double l, double &x, double &y, double &th){
    double l2 = l*l;
    double l3 = l2*l;
    
    x = coef[0] + coef[1]*l + coef[2]*l2 + coef[3]*l3;
    y = coef[4] + coef[5]*l + coef[6]*l2 + coef[7]*l3;
    th= atan2f64(coef[5] + 2*coef[6]*l + 3*coef[7]*l2, 
                 coef[1] + 2*coef[2]*l + 3*coef[3]*l2);
}

void pathGenerator(const double coef[],const uint32_t numPoints, double x[], double y[], double th[]){
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
        cout << "Caso especial 1\n";
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
        cout << "Caso especial 2\n";
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
        cout << "Caso especial 3\n";
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
        cout << "Caso geral\n";
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
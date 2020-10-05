// Enabling the B0-based remote API - client side
// REF.: https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm

#include "b0RemoteApi.h"

#include "pid.hpp"
#include "utils.hpp"
#include "common.hpp"

#include <iostream>
#include <vector>

#define N 200 //quantidade de pontos gerados em um caminho
const int RED[] = {255,0,0};
const int BLUE[]= {0,0,255};

using namespace std;

int  drawingPoints(const float x[], const float y[], const float z, const int color[]);
int  drawingSphere(const float xc, const float yc, const float zc, 
                   const int color[], const float size);

b0RemoteApi* cl=NULL;
int main(){
    double path_coef[8];
    float x_v[N], y_v[N], th_v[N];
    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    std::cout << "Conectado!\n";
    cl=&client;

    interPoly3(0,0,0, 2.5,2.5, 0, path_coef);
    pathGenerator(path_coef, N, x_v, y_v, th_v);
    
    int points_handle = drawingPoints(x_v, y_v, 0.1F, COLORS::BLACK);
    int target_handle = drawingSphere(0,0,0, COLORS::BLUE, 0.1);

    client.simxSleep(10*1000);
    // removeing drawing objects
    client.simxRemoveDrawingObject(points_handle, client.simxServiceCall());
    client.simxRemoveDrawingObject(target_handle, client.simxServiceCall());
    return 0;
}


int  drawingPoints(const float x[], const float y[], const float z, const int color[])
{
    std::vector<msgpack::object>* res;
    float points[N*3];
    for(int i = 0; i < N; i++){
        points[i*3]   = x[i]; // x 
        points[i*3+1] = y[i]; // y
        points[i*3+2] = z;    // z
    }
    res = cl->simxAddDrawingObject_points(1, color, points, N*3, cl->simxServiceCall());
    return b0RemoteApi::readInt(res,1);
}

int  drawingSphere(const float xc, 
                   const float yc, 
                   const float zc, 
                   const int color[], 
                   const float size)
{   
    std::vector<msgpack::object>* res;
    float coords[] = {xc,yc,zc};
    res = cl->simxAddDrawingObject_spheres(size, color, coords, 3, cl->simxServiceCall());
    return b0RemoteApi::readInt(res,1);
}
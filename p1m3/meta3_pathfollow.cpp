// Enabling the B0-based remote API - client side
// REF.: https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm

#include "b0RemoteApi.h"

#include "utils.hpp"
#include "common.hpp"
#include "kinematicsControllers.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"

#include <iostream>
#include <vector>
#include <string>
#include <omp.h>

using namespace std;
namespace plt = matplotlibcpp;

#define N 200 //quantidade de pontos gerados em um caminho

int  drawingPoints(const float x[], const float y[], const float z, const int color[]);
int  drawingSphere(const float xc, const float yc, const float zc, 
                   const int color[], const float size);

b0RemoteApi* cl = NULL;
int main(){
    std::vector<float> pioneer_pos(3), pioneer_ori(3);
    std::vector<float> target_pos(3), target_ori(3);
    std::vector<float> time_v;
    std::vector<float> w_l_v, w_r_v;
    std::vector<float> vel_v;
    float pioneer_velocity;
    int pioneer, leftMotor, rightMotor, target;
    double v, w;
    double currTime, startTime;
    double coef[NUM_PARAMETERS], ang_error, lin_error;
    float w_l, w_r; //velocidade angular(w [rad/s]) das rodas esquerda e direita
    
    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if(!r)std::cerr<<"Falha ao carregar o cenário!\n";
    std::string scene = string(cCurrentPath) + string("/scenes/pathfollow.ttt");
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";
    cl=&client;
    
    pioneer   = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx",client.simxServiceCall()),1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor",client.simxServiceCall()),1);
    rightMotor= b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor",client.simxServiceCall()),1);
    target    = b0RemoteApi::readInt(client.simxGetObjectHandle("Target",client.simxServiceCall()),1);

    
    b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos,1);    
    b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos,1);
    b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori,1);    
    b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(target, -1, client.simxServiceCall()), target_ori,1);    
    
    pathComputing(pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], target_pos[0], target_pos[1], target_ori[2], coef);

    double K_lin = 2.0;
    double K_ang = 4.0;
    PathFollowController pathfollow(K_ang, K_lin, coef);

    // drawing the path
    float x[N], y[N], th[N];
    pathGenerator(coef, N, x, y, th);
    int lines = drawingPoints(x, y, 0.01, COLORS::BLUE);

    startTime = omp_get_wtime();
    while(true)
    {   
        currTime = omp_get_wtime() - startTime;
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos,1);    
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos,1);
        b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori,1);    

        v = 0.4;
        bool done = pathfollow.step(pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], 
                                    v, w, lin_error, ang_error);
        // converte saidas do controlador para velocidades dos motores
        pioneer_model(v, w, w_r, w_l);
        
        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor,  w_l, client.simxServiceCall());

        // plot zone
        // time_v.push_back(currTime);
        // vel_v.push_back(v);
        // // Clear previous plot
        // plt::clf();
        // plt::plot(time_v, vel_v, "-b");
        // plt::named_plot("Velocity", time_v, vel_v, "-b");
        // // plt::named_plot("Linear Error", time_v, lin_error_v, "-g");
        // // Add graph title
        // // plt::title("Sample figure");
        // plt::xlabel("t[s]");
        // // Enable legend.
        // // plt::legend();
        // plt::grid(true);
        // // Display plot continuously
        // plt::pause(0.01);
        
        if(done){
            std::cout << "Chegou no alvo!\n";
            break;
        }
        usleep(1000*100);
    }
    // plt::show();

    client.simxSetJointTargetVelocity(rightMotor, 0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor,  0, client.simxServiceCall());
    client.simxStopSimulation(client.simxServiceCall());

    client.simxRemoveDrawingObject(lines, client.simxServiceCall());
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
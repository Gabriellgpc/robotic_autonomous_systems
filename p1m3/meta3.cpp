// Enabling the B0-based remote API - client side
// REF.: https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm

#include "b0RemoteApi.h"

#include "utils.hpp"
#include "common.hpp"
#include "kinematicsControllers.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"

#include <iostream>
#include <vector>
#include <omp.h>

#define N 200 //quantidade de pontos gerados em um caminho

using namespace std;
namespace plt = matplotlibcpp;

// void getConfig(int handle, );
int  drawingPoints(const float x[], const float y[], const float z, const int color[]);
int  drawingSphere(const float xc, const float yc, const float zc, 
                   const int color[], const float size);
void pioneer_model(float v, float w, float &w_right, float &w_left);

b0RemoteApi* cl=NULL;
int main(){
    int pioneer, leftMotor, rightMotor;
    int target;

    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";
    cl=&client;
    
    pioneer   = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx",client.simxServiceCall()),1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor",client.simxServiceCall()),1);
    rightMotor= b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor",client.simxServiceCall()),1);
    target    = b0RemoteApi::readInt(client.simxGetObjectHandle("Target",client.simxServiceCall()),1);

    PositionController posController(0.1,0.01,0, 
                                     0.5,0.15,0);

    std::vector<float> pioneer_pos(3);
    std::vector<float> pioneer_ori(3);
    std::vector<float> target_pos(3);
    std::vector<float> ang_error_v, lin_error_v;
    std::vector<float> time_v;
    std::vector<float> w_l_v, w_r_v;
    double v, w;
    double lin_error, ang_error;
    float w_l, w_r; //velocidade angular(w [rad/s]) das rodas esquerda e direita
    double currTime;
    double startTime = omp_get_wtime();
    while(true)
    {   
        currTime = omp_get_wtime() - startTime;
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos,1);    
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos,1);
        b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori,1);    

        bool done = posController.step(target_pos[0], target_pos[1], 
                                       pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], 
                                       v, w, 
                                       lin_error, ang_error);
        if(done){
            std::cout << "Chegou no alvo!\n";
            break;
        }

        // converte saidas do controlador para velocidades dos motores
        pioneer_model(v, w, w_r, w_l);

        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor,  w_l, client.simxServiceCall());

        // plot zone
        time_v.push_back(currTime);
        ang_error_v.push_back(ang_error);
        lin_error_v.push_back(lin_error);
        // Clear previous plot
        plt::clf();
        // plt::plot(time_v, ang_error_v, "-b");
        plt::named_plot("Angular Error", time_v, ang_error_v, "-b");
        plt::named_plot("Linear Error", time_v, lin_error_v, "-g");
        // Add graph title
        plt::title("Sample figure");
        // Enable legend.
        plt::legend();
        plt::grid(true);
        // Display plot continuously
        plt::pause(0.01);    
        // printf("Tempo para plotar: %0.5f\n",tok - tik);

        // sleep for 100ms
        // usleep(100*1000);
    }

    client.simxSetJointTargetVelocity(rightMotor, 0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor,  0, client.simxServiceCall());
    client.simxStopSimulation(client.simxServiceCall());
    return 0;
}

void pioneer_model(float v, float w, float &w_right, float &w_left)
{
    const static float b = 0.331;     //wheel axis distance [m]
    const static float r = 0.09751;   //wheel radius [m]
    w_right = (1.0/r)*v + (b/(2.0*r))*w;
    w_left  = (1.0/r)*v - (b/(2.0*r))*w;
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
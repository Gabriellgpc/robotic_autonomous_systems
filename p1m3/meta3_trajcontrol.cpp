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
    std::vector<float> pioneer_pos(3), pioneer_ori(3), pioneer_velocity(3);
    std::vector<float> target_pos(3), target_ori(3);
    std::vector<float> time_vec;
    std::vector<float> w_l_v, w_r_v;
    std::vector<float> vref_vec, dvref_vec, xref_vec, yref_vec, thref_vec;
    std::vector<float> x_vec, y_vec, v_vec;
    double xref, yref, vref, dvref, wc;

    int pioneer, leftMotor, rightMotor, target;
    double v, w, pioneer_speed;
    double currTime, startTime;
    double coef[NUM_PARAMETERS];
    float w_l, w_r; //velocidade angular(w [rad/s]) das rodas esquerda e direita
    
    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if(!r)std::cerr<<"Falha ao carregar o cenário!\n";
    std::string scene = string(cCurrentPath) + string("/scenes/trajcontrol.ttt");
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";
    cl=&client;
    
    pioneer   = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx",client.simxServiceCall()),1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor",client.simxServiceCall()),1);
    rightMotor= b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor",client.simxServiceCall()),1);
    target    = b0RemoteApi::readInt(client.simxGetObjectHandle("Target",client.simxServiceCall()),1);

    float Kp = 1.0;
    float Kd = 1.0;
    TrajController trajcontrol(Kd, Kp);
    
    b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos,1);    
    b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos,1);
    b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori,1);    
    b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(target, -1, client.simxServiceCall()), target_ori,1);    
    
    pathComputing(pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], target_pos[0], target_pos[1], target_ori[2], coef);

    trajcontrol.setTrajectory(coef, 30.0);
    // drawing the path
    std::vector<float> xps(N), yps(N), thps(N);
    pathGenerator(coef, N, xps.data(), yps.data(), thps.data());
    int lines = drawingPoints(xps.data(), yps.data(), 0.01, COLORS::BLUE);

    startTime = omp_get_wtime();
    while(true)
    {       
        currTime = omp_get_wtime() - startTime;
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos,1);    
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos,1);
        b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori,1);    

        b0RemoteApi::readFloatArray(client.simxGetObjectVelocity(pioneer, client.simxServiceCall()), pioneer_velocity,1);
        double config[] = {pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], pioneer_velocity[0], pioneer_velocity[1]};
        bool done = trajcontrol.step(config, 
                                     xref, yref, vref, dvref, wc,
                                     v, w);
        // converte saidas do controlador para velocidades dos motores
        pioneer_model(v, w, w_r, w_l);
        // pioneer_model(0.4, 0, w_r, w_l);
        // printf("Velocity: (%.4f, %.4f, %.4f)\n", pioneer_velocity[0], pioneer_velocity[1], pioneer_velocity[2]);
        
        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor,  w_l, client.simxServiceCall());

        // plot zone
        time_vec.push_back(currTime);
        vref_vec.push_back(vref);
        dvref_vec.push_back(dvref);
        xref_vec.push_back(xref);
        yref_vec.push_back(yref);
        x_vec.push_back(pioneer_pos[0]);
        y_vec.push_back(pioneer_pos[1]);
        v_vec.push_back( sqrt(pow(pioneer_velocity[0],2) + pow(pioneer_velocity[1],2)) );
        
        // // Clear previous plot
        plt::clf();
        
        plt::subplot(2,1,1);
        plt::named_plot("Velocity Ref.", time_vec, vref_vec, "-k");
        plt::named_plot("Velocity", time_vec, v_vec, "-b");
        // plt::named_plot("Aceleration Ref.", time_vec, dvref_vec, "-k");
        plt::legend();
        plt::xlabel("t[s]");
        plt::grid(true);

        plt::subplot(2,1,2);
        plt::named_plot("Path", xps, yps, "-k");
        plt::named_plot("Pos Ref.", xref_vec, yref_vec, "*b");
        plt::named_plot("Pioneer", x_vec, y_vec, "or");
        plt::ylabel("y[m]");
        plt::xlabel("x[m]");
        plt::legend();
        plt::grid(true);
        
        // // Display plot continuously
        plt::pause(0.01);    
        
        if(done){
            std::cout << "Chegou no alvo!\n";
            break;
        }
        // usleep(1000*100);
    }
    plt::show();

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
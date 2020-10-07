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

int main(){
    int pioneer, leftMotor, rightMotor;
    int target;

    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if(!r)std::cerr<<"Falha ao carregar o cenário!\n";
    std::string scene = string(cCurrentPath) + string("/scenes/poscontrol.ttt");
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";
    
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
        // Add graph title
        plt::subplot(2,1,1);
        plt::named_plot("Angular Error", time_v, ang_error_v, "-b");
        plt::ylabel("[rad]");
        plt::legend();
        plt::grid(true);
        
        plt::subplot(2,1,2);
        plt::named_plot("Linear Error", time_v, lin_error_v, "-g");
        plt::ylabel("[m]");
        plt::xlabel("t[s]");
        // Enable legend.
        plt::legend();
        plt::grid(true);
        // Display plot continuously
        plt::pause(0.01);    
        
        if(done){
            std::cout << "Chegou no alvo!\n";
            break;
        }
    }

    plt::show();

    client.simxSetJointTargetVelocity(rightMotor, 0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor,  0, client.simxServiceCall());
    client.simxStopSimulation(client.simxServiceCall());
    return 0;
}
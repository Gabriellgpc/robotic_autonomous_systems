// Enabling the B0-based remote API - client side
// REF.: https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm

#include "common.hpp"

#include <b0RemoteApi.h>
#include <utils.hpp>
#include <kinematicsControllers.hpp>
#include <matplotlibcpp.h>

#include <iostream>
#include <vector>
#include <string>
#include <omp.h>
#include <thread>

using namespace std;
namespace plt = matplotlibcpp;

#define Kp_lin 0.1
#define Ki_lin 0.01
#define Kd_lin 0.0
#define Kp_ang 0.5
#define Ki_ang 0.15
#define Kd_ang 0.0

bool finished = false, inited = false;
char cCurrentPath[FILENAME_MAX];
std::vector<float> pioneer_pos(3);
std::vector<float> pioneer_ori(3);
std::vector<float> target_pos(3);
std::vector<float> lin_error_vec, ang_error_vec;
std::vector<float> x_vec, y_vec, wl_vec, wr_vec, v_vec, w_vec;
std::vector<float> time_vec;

int pioneer, leftMotor, rightMotor;
int target;
double v, w;
double lin_error, ang_error;
float w_l, w_r; //velocidade angular(w [rad/s]) das rodas esquerda e direita
double currTime;

void plotting()
{
    std::vector<float> x_target, y_target;

    while (!finished)
    {   
        if(!inited){
            std::this_thread::yield();
            continue;
        }
        // data update
        time_vec.push_back(currTime);
        x_vec.push_back(pioneer_pos[0]);
        y_vec.push_back(pioneer_pos[1]);
        lin_error_vec.push_back(lin_error);
        ang_error_vec.push_back(ang_error);
        v_vec.push_back(v);
        w_vec.push_back(w);
        wl_vec.push_back(w_l);
        wr_vec.push_back(w_r);
        
        x_target.push_back(target_pos[0]);
        y_target.push_back(target_pos[1]);
        
        // Clear previous plot
        plt::figure(1);
        plt::clf();
        plt::named_plot("Pioneer Position", x_vec, y_vec, "-*r");
        plt::named_plot("Target", x_target, y_target, "*b");
        plt::legend();
        plt::grid(true);
        plt::xlabel("x[m]");
        plt::ylabel("y[m]");

        plt::figure(3);
        plt::clf();
        plt::subplot(2, 1, 1);
        plt::named_plot("Linear Velocity", time_vec, v_vec, "-b");
        plt::title("V(t) and W(t)");
        plt::legend();
        plt::grid(true);
        plt::ylabel("[m/s]");

        plt::subplot(2, 1, 2);
        plt::named_plot("Angular Velocity", time_vec, w_vec, "-k");
        plt::legend();
        plt::grid(true);
        plt::ylabel("[rad]");
        plt::xlabel("t[s]");

        plt::figure(4);
        plt::clf();
        plt::subplot(2,1,1);
        plt::named_plot("Angular Error", time_vec, ang_error_vec, "-b");
        plt::legend();
        plt::grid(true);    
        plt::ylabel("[rad]");

        plt::subplot(2,1,2);
        plt::named_plot("Linear Error", time_vec, lin_error_vec, "-k");
        plt::legend();
        plt::grid(true);
        plt::ylabel("[m]");
        plt::xlabel("t[s]");

        // plt::figure(5);
        // plt::clf();
        // plt::named_plot("$\\omega_{left}(t)$", time_vec, wl_vec, "-b");
        // plt::named_plot("$\\omega_{right}(t)$", time_vec, wr_vec, "-g");
        // plt::legend();
        // plt::grid(true);
        // plt::ylabel("[rad/s]");
        // plt::xlabel("t[s]");

        plt::pause(0.01);
    }
    plt::show();
};

int main()
{   
    std::thread thr_pyplot(plotting);

    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn", "b0RemoteApiAddOn");
    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if (!r){
        std::cerr << "Falha ao carregar o cenário!\n";
    }
    std::string scene = string(cCurrentPath) + string("/scenes/poscontrol.ttt");
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";

    pioneer = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx", client.simxServiceCall()), 1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor", client.simxServiceCall()), 1);
    rightMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor", client.simxServiceCall()), 1);
    target = b0RemoteApi::readInt(client.simxGetObjectHandle("Target", client.simxServiceCall()), 1);

    PositionController posController(Kp_lin, Ki_lin, Kd_lin,
                                     Kp_ang, Ki_ang, Kd_ang);

    double startTime = omp_get_wtime();
    while (true)
    {
        currTime = omp_get_wtime() - startTime;
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos, 1);
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos, 1);
        b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori, 1);

        finished = posController.step(target_pos[0], target_pos[1],
                                      pioneer_pos[0], pioneer_pos[1], pioneer_ori[2],
                                      v, w,
                                      lin_error, ang_error);
        // converte saidas do controlador para velocidades dos motores
        pioneer_model(v, w, w_r, w_l);

        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor, w_l, client.simxServiceCall());

        if(!inited)
            inited = true;
        if(finished)
        {
            std::cout << "The end!\n";
            break;
        }
    }
    thr_pyplot.join();

    client.simxSetJointTargetVelocity(rightMotor, 0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor, 0, client.simxServiceCall());
    client.simxStopSimulation(client.simxServiceCall());

    return 0;
}
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
#include <thread>         // std::thread

using namespace std;
namespace plt = matplotlibcpp;

#define N 200 //quantidade de pontos gerados em um caminho

int  drawingPoints(const float x[], const float y[], const float z, const int color[]);
int  drawingSphere(const float xc, const float yc, const float zc, 
                   const int color[], const float size);

char cCurrentPath[FILENAME_MAX];
std::vector<float> pioneer_pos(3), pioneer_ori(3);
std::vector<float> target_pos(3), target_ori(3);
std::vector<float> time_vec, ang_error_vec, lin_error_vec;
std::vector<float> wl_vec, wr_vec;
std::vector<float> v_vec, w_vec;
std::vector<float> x_path(N), y_path(N), th_path(N);
std::vector<float> x_vec, y_vec, xref_vec, yref_vec;
float pioneer_velocity;
int pioneer, leftMotor, rightMotor, target;
double v, w, xref, yref, th_ref;
double currTime, startTime;
double coef[NUM_PARAMETERS], ang_error, lin_error;
float w_l, w_r; //velocidade angular(w [rad/s]) das rodas esquerda e direita
bool finished=false, inited = false;

void plotting(){
    while(!finished)
    {   
        if(!inited){
            std::this_thread::yield();
            continue;
        }
        // salva variaveis para plotar
        time_vec.push_back(currTime);
        x_vec.push_back(pioneer_pos[0]);
        y_vec.push_back(pioneer_pos[1]);
        xref_vec.push_back(xref);
        yref_vec.push_back(yref);
        ang_error_vec.push_back(ang_error);
        lin_error_vec.push_back(lin_error);
        v_vec.push_back(v);
        w_vec.push_back(w);

        //Clear previous plot
        plt::figure(1);
        plt::clf();
        plt::named_plot("Path", x_path, y_path, "-k");
        plt::named_plot("Pioneer", x_vec, y_vec, "or");
        plt::named_plot("Referência", xref_vec, yref_vec, "*b");
        plt::legend();
        plt::grid(true);
        plt::xlabel("x[m]");
        plt::ylabel("y[m]");

        plt::figure(2);
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


        plt::pause(0.01);
    }
    plt::show();
};

b0RemoteApi* cl = NULL;
int main(){
    std::thread thr_pyplot(plotting);
    
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

    double K_lin = 0.1;
    double K_ang = 1.0;
    PathFollowController pathfollow(K_ang, K_lin, coef);


    printf("Path Parameters:\n a0 = %.4lf \n a1 = %.4lf \n a2 = %.4lf \n a3 = %.4lf \n b0 = %.4lf \n b1 = %.4lf \n b2 = %.4lf \n b3 = %.4lf\n",
           coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7]);
    // drawing the path
    pathGenerator(coef, N, x_path.data(), y_path.data(), th_path.data());
    int lines = drawingPoints(x_path.data(), y_path.data(), 0.01, COLORS::BLUE);

    startTime = omp_get_wtime();
    double tik, tok;
    while(true)
    {   
        tik = omp_get_wtime();

        currTime = omp_get_wtime() - startTime;
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos,1);    
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos,1);
        b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori,1);    

        v = 0.3;
        finished = pathfollow.step(pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], v, w,
                                   xref, yref, th_ref,lin_error, ang_error);
        // converte saidas do controlador para velocidades dos motores
        pioneer_model(v, w, w_r, w_l);
        
        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor,  w_l, client.simxServiceCall());

        tok = omp_get_wtime();
        
        if(!inited)
            inited = true;    
        if(finished){
            std::cout << "The end!\n";
            break;
        }
    }
    
    thr_pyplot.join();

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
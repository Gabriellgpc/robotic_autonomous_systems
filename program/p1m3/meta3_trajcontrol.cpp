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

#define Kp 0.8 // 1.0
#define Kd 0.8 // 0.2
#define VMAX 0.5
#define N 200 //quantidade de pontos gerados em um caminho

int drawingPoints(const float x[], const float y[], const float z, const int color[]);
int drawingSphere(const float xc, const float yc, const float zc,
                  const int color[], const float size);

b0RemoteApi *cl = NULL;
char cCurrentPath[FILENAME_MAX];
std::vector<float> time_vec, ang_error_vec, lin_error_vec;
std::vector<float> wl_vec, wr_vec;
std::vector<float> x_path(N), y_path(N), th_path(N);
std::vector<float> x_vec, y_vec, v_vec;
std::vector<float> xref_vec, yref_vec, vref_vec, dvref_vec;

std::vector<float> pioneer_pos(3), pioneer_ori(3), pioneer_velocity(3);
std::vector<float> target_pos(3), target_ori(3);
double xref, yref, vref, dvref, wc;
int pioneer, leftMotor, rightMotor, target;
double v, w, pioneer_speed;
double currTime, startTime;
double coef[NUM_PARAMETERS];
float w_l, w_r; //velocidade angular(w [rad/s]) das rodas esquerda e direita
bool finished = false, inited = false;

void plotting()
{
    while (!finished)
    {   
        if(!inited){
            std::this_thread::yield();
            continue;
        }
        time_vec.push_back(currTime);
        
        x_vec.push_back(pioneer_pos[0]);
        y_vec.push_back(pioneer_pos[1]);
        v_vec.push_back(sqrt(pow(pioneer_velocity[0], 2) + pow(pioneer_velocity[1], 2)));
        
        vref_vec.push_back(vref);
        xref_vec.push_back(xref);
        yref_vec.push_back(yref);
        
        dvref_vec.push_back(dvref);
        // wl_vec.push_back(w_l);
        // wr_vec.push_back(w_r);
        
        plt::figure(1);
        plt::clf();
        plt::named_plot("Velocity Ref.", time_vec, vref_vec, "-k");
        plt::named_plot("Velocity", time_vec, v_vec, "-b");
        plt::legend();
        plt::ylabel("[m/s]");
        plt::xlabel("t[s]");
        plt::grid(true);

        plt::figure(2);
        plt::clf();
        plt::named_plot("Path", x_path, y_path, "-k");
        plt::named_plot("Pos Ref.", xref_vec, yref_vec, "*b");
        plt::named_plot("Pioneer", x_vec, y_vec, "or");
        plt::ylabel("y[m]");
        plt::xlabel("x[m]");
        plt::legend();
        plt::grid(true);

        // Display plot continuously
        plt::pause(0.01);
    }
    plt::show();
};

int main()
{
    std::thread thr_pyplot(plotting);

    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn", "b0RemoteApiAddOn");
    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if (!r)
        std::cerr << "Falha ao carregar o cenário!\n";
    std::string scene = string(cCurrentPath) + string("/scenes/trajcontrol.ttt");
    
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";
    cl = &client;

    pioneer = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx", client.simxServiceCall()), 1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor", client.simxServiceCall()), 1);
    rightMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor", client.simxServiceCall()), 1);
    target = b0RemoteApi::readInt(client.simxGetObjectHandle("Target", client.simxServiceCall()), 1);
    
    TrajController trajcontrol(Kd, Kp);

    b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos, 1);
    b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos, 1);
    b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori, 1);
    b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(target, -1, client.simxServiceCall()), target_ori, 1);

    pathComputing(pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], target_pos[0], target_pos[1], target_ori[2], coef);

    trajcontrol.setTrajectory(coef, VMAX);
    
    // drawing the path
    pathGenerator(coef, N, x_path.data(), y_path.data(), th_path.data());
    int lines = drawingPoints(x_path.data(), y_path.data(), 0.01, COLORS::BLUE);

    printf("Path Parameters:\n a0 = %.4lf \n a1 = %.4lf \n a2 = %.4lf \n a3 = %.4lf \n b0 = %.4lf \n b1 = %.4lf \n b2 = %.4lf \n b3 = %.4lf\n",
           coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7]);

    startTime = omp_get_wtime();
    while (true)
    {
        currTime = omp_get_wtime() - startTime;
        b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos, 1);
        b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori, 1);
        b0RemoteApi::readFloatArray(client.simxGetObjectVelocity(pioneer, client.simxServiceCall()), pioneer_velocity, 1);

        double config[] = {pioneer_pos[0], pioneer_pos[1], pioneer_ori[2], pioneer_velocity[0], pioneer_velocity[1]};
        finished = trajcontrol.step(config,
                                    xref, yref, vref, dvref, wc,
                                    v, w);
        // converte saidas do controlador para velocidades dos motores
        pioneer_model(v, w, w_r, w_l);

        if(!inited)inited=true;
        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor,  w_l, client.simxServiceCall());

        if (finished)
        {
            std::cout << "Finished!\n";
            break;
        }
        // usleep(30.0*1000.0);
    }
    thr_pyplot.join();

    client.simxSetJointTargetVelocity(rightMotor, 0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor, 0, client.simxServiceCall());
    client.simxStopSimulation(client.simxServiceCall());
    client.simxRemoveDrawingObject(lines, client.simxServiceCall());
    return 0;
}

int drawingPoints(const float x[], const float y[], const float z, const int color[])
{
    std::vector<msgpack::object> *res;
    float points[N * 3];
    for (int i = 0; i < N; i++)
    {
        points[i * 3] = x[i];     // x
        points[i * 3 + 1] = y[i]; // y
        points[i * 3 + 2] = z;    // z
    }
    res = cl->simxAddDrawingObject_points(1, color, points, N * 3, cl->simxServiceCall());
    return b0RemoteApi::readInt(res, 1);
}

int drawingSphere(const float xc,
                  const float yc,
                  const float zc,
                  const int color[],
                  const float size)
{
    std::vector<msgpack::object> *res;
    float coords[] = {xc, yc, zc};
    res = cl->simxAddDrawingObject_spheres(size, color, coords, 3, cl->simxServiceCall());
    return b0RemoteApi::readInt(res, 1);
}
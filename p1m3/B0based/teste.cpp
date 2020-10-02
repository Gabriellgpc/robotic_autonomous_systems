// Enabling the B0-based remote API - client side
// REF.: https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm

#include "b0RemoteApi.h"

#include <iostream>
#include <cmath>

using namespace std;

// V = W . R

void pioneer(double V,double W, double &wd, double &we){
    float R = V/W;
    float b = 0.331;   //wheel axis distance [m]
    float r = 0.09751; //wheel radius [m]
    wd = W*(R + b/2.0)/r;
    we = wd * (R - b/2.0)/(R+b/2.0);
}

const double time_stop = 35; //tempo de simulação [s]
const double R = 2.0; //raio de giro [m]

int main(){
    double simTime = 0.0;
    double wd, we;
    b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn","b0RemoteApiAddOn");
    cout << "Conectado!\n";
    
    client.simxAddStatusbarMessage("Cliente Conectado!",client.simxDefaultPublisher());
    client.simxStartSimulation(client.simxServiceCall());
    
    int lmotor_handle  = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor",client.simxServiceCall()),1);
    int rmotor_handle  = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor",client.simxServiceCall()),1);
    int pioneer_handle = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx",client.simxServiceCall()),1);

    double W = (2.0*M_PIf64)/time_stop;
    pioneer(R*W, W, wd, we);

    cout << "V = " << R*W << " W = " << W << endl;    
    cout << "wd = " << wd << " | we = " << we << endl;

    double beginTime = client.simxGetTimeInMs() / 1000.0;
    while(simTime <= time_stop){
        client.simxSetJointTargetVelocity(rmotor_handle, wd, client.simxServiceCall());
        client.simxSetJointTargetVelocity(lmotor_handle, we, client.simxServiceCall());
        client.simxSleep(100.0);
        simTime = client.simxGetTimeInMs()/1000.0 - beginTime;
        cout << "time:" << simTime << endl;
    }
    client.simxStopSimulation(client.simxServiceCall());
    client.simxSleep(500.0);

    return 0;
}
#include "extApi.h"

#include <iostream>
#include <unistd.h>

using namespace std;

int main(){
    simxInt LwMotor_handle, RwMotor_handle, robot_handle;
    simxInt clientID = simxStart("127.0.0.1", 19999,true, true, 5000,5);

    if( clientID != 0){
        cout << "Erro: Cliente não conectado\n";
    }else{
        cout << "Conexão estabelecida!\n";
    }
    simxInt returnLM, returnRM, returnR;
    returnLM = simxGetObjectHandle(clientID,"Pioneer_p3dx_leftMotor", &LwMotor_handle, simx_opmode_oneshot_wait);
    returnRM = simxGetObjectHandle(clientID,"Pioneer_p3dx_rightMotor",&RwMotor_handle,simx_opmode_oneshot_wait);
    returnR  = simxGetObjectHandle(clientID,"Pioneer_p3dx", &robot_handle, simx_opmode_oneshot_wait);    

    
        
    cout << "Desconectando...\n";
    extApi_sleepMs(500);
    
    //  Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    simxInt pingTime;
    simxGetPingTime(clientID, &pingTime);

    // Now close the connection to CoppeliaSim:
    simxFinish(clientID);
    cout << "Desconectado!\n";

    return 0;
}
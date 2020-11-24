#pragma once
#include <b0RemoteApi.h>             //para comunicação com o coppeliaSim via Blue-zero
#include <utils.hpp>                 //pioneer_model
#include <kinematicsControllers.hpp> //PositionController
#include <configSpaceTools.hpp>      //Polygon2D, Robot, World, Vector2D, Config

#include <thread> //std::thread
#include <vector> //std::vector
#include <mutex>  // std::mutex
#include <string>

const std::string nodeName = std::string("b0RemoteApi_CoppeliaSim-addOn");
const std::string channelName = std::string("b0RemoteApiAddOn");

#define Kp_lin 0.1
#define Ki_lin 0.01
#define Kd_lin 0.0
#define Kp_ang 0.5
#define Ki_ang 0.15
#define Kd_ang 0.0

class Simulation_p3m1
{
public:
    Simulation_p3m1();
    ~Simulation_p3m1();

    //método que lançará as threads e iniciará a simulação
    void start_simulation(const std::string &scene, const float &time_to_stop);

private:
    std::thread thr_plotter;  //thread responsável pela plotagem dinâmica
    std::thread thr_receiver; //thread responsável por receber os dados da simulação
    // std::thread thr_sender;     //thread responsável por enviar os comandos para o simulador

    std::mutex mtx_currConfig; //mutex para acesso a configuração atual

    b0RemoteApi *client;

    Config currConfig;

    float time_to_stop; //[s]
    bool to_stop = false;

    int handle_pioneer;
    int handle_leftMotor;
    int handle_rightMotor;
    int handle_target;
    int handle_sensor;

    void _plotter_routine();
    void _receiver_routine();
    void _sender_routine();

    friend void _receiver_func(void *X);
    friend void _plotter_func(void *X);
    friend void _sender_func(void *X);

    PositionController stab_control;
};
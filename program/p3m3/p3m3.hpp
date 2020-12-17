#pragma once
#include <b0RemoteApi.h>             //para comunicação com o coppeliaSim via Blue-zero
#include <utils.hpp>                 //pioneer_model
#include <configSpaceTools.hpp>      //Polygon2D, Robot, World, Vector2D, Config
#include <occupating_grid.hpp>       //OccupationGrid, CellGrid, ProximitySensorInfo

#include <thread> //std::thread
#include <vector> //std::vector
#include <mutex>  //std::mutex
#include <string> //std::string
#include <queue>  //std:queue
#include <atomic> //std::atomic

//limiar usado para considerar uma celula ocupada
constexpr double occupating_thr = 0.1;
//Numero de sensores
constexpr int N_SENSORS = 4;
const std::string nodeName = std::string("b0RemoteApi_CoppeliaSim-addOn");
const std::string channelName = std::string("b0RemoteApiAddOn");


struct MyDatas
{
    Config q;
};

class Simulation_p3m3
{
public:
    Simulation_p3m3();
    ~Simulation_p3m3();
    //método que lançará as threads e iniciará a simulação
    void start_simulation(const std::string &scene, const float &time_to_stop);
    void stop_simulation();
private:
    std::thread thr_plotter;  //thread responsável pela plotagem dinâmica
    std::thread thr_receiver; //thread responsável por receber os dados da simulação
    std::mutex  mtx; //mutex para acesso a configuração atual

    b0RemoteApi *client;

    std::atomic<float> time_to_stop; //[s]
    std::atomic<bool> to_stop;

    int handle_pioneer;
    int handle_leftMotor;
    int handle_rightMotor;

    void _plotter_routine();
    void _receiver_routine();

    void _readDatas();

    std::queue<MyDatas> myqueue;
    OccupationGrid my_occup_grid;

    friend void _receiver_func(void *X);
    friend void _plotter_func(void *X);
};

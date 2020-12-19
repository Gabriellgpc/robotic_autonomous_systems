#pragma once
#include <b0RemoteApi.h>             //para comunicação com o coppeliaSim via Blue-zero
#include <utils.hpp>                 //pioneer_model
#include <configSpaceTools.hpp>      //Polygon2D, Robot, World, Vector2D, Config
#include <occupating_grid.hpp>       //OccupationGrid, OccupationCell, ProximitySensorInfo
#include <planning.hpp>              //manhattan, depthfirst, RegularGrid, CellGrid
#include <kinematicsControllers.hpp> //PathFollowController

#include <thread> //std::thread
#include <vector> //std::vector
#include <mutex>  //std::mutex
#include <string> //std::string
#include <queue>  //std:queue
#include <atomic> //std::atomic
#include <list>   //std::list

//limiar usado para considerar uma celula ocupada
constexpr double occupating_thr = 0.1;
//Numero de sensores
const std::string nodeName = std::string("b0RemoteApi_CoppeliaSim-addOn");
const std::string channelName = std::string("b0RemoteApiAddOn");

constexpr double K_ang = 0.5;
constexpr double K_lin = 1.0;

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
    std::thread thr_controller; //thread responsável por receber os dados da simulação
    std::mutex  mtx; //mutex para acesso a configuração atual

    b0RemoteApi *client;

    std::atomic<float> time_to_stop; //[s]
    std::atomic<bool> to_stop;

    int handle_pioneer;
    int handle_leftMotor;
    int handle_rightMotor;
    int handle_target;

    void _plotter_routine();
    void _receiver_routine();

    void _readDatas();

    std::queue<MyDatas> myqueue;
    OccupationGrid my_prob_occup_grid;
    RegularGrid    my_regular_grid;
    Robot          my_robot;
    std::list<CellGrid> my_path_cell;
    PathFollowController my_path_follower;

    friend void _receiver_func(void *X);
    friend void _plotter_func(void *X);
};

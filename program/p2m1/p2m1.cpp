#include "p2m1.hpp"  //polygon2D_to_Vector2D

#include <gnuplot-iostream.h>   //Gnuplot
#include <configSpaceTools.hpp> //Polygon2D, Robot, World, Vector2D
#include <b0RemoteApi.h>  //para comunicação com o coppeliaSim via Blue-zero
#include <utils.hpp> //pioneer_model

#include <iostream> //cout
#include <stdio.h>  //printf
#include <cstdio>   //FILENAME_MAX, printf
#include <unistd.h> //getcwd
#include <string>  //string
#include <thread>  //thread
#include <omp.h>   //omp_get_wtime()

// #define PLOT_CONFIG_SPACE

unsigned int SAMPLES  = 200;
double pioneer_radius = 0.52/2.0;
unsigned int n_vertices = 8;
#define GetCurrentDir getcwd
#define STOP_TIME 120.0 //segundos

using namespace std;

void living_plot();
void _init(int argc, char **argv);

b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn", "b0RemoteApiAddOn");
Robot robot(Config(-3.0,-2.0,M_PI/4.0), Polygon2D::circle_to_polygon2D(pioneer_radius, n_vertices));
World W(robot);
char cCurrentPath[FILENAME_MAX];
bool is_finished = false;

int main(int argc, char **argv)
{
    std::thread thr_plot; 
    std::vector<float> pioneer_pos, pioneer_ori;
    int pioneer, leftMotor, rightMotor;   
    float w_r, w_l;
    _init(argc, argv);
    
    pioneer = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx", client.simxServiceCall()), 1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor", client.simxServiceCall()), 1);
    rightMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor", client.simxServiceCall()), 1);

    thr_plot = std::thread(living_plot);
    Config curr_config;
    double time = 0, start_time = omp_get_wtime();
    do{
        time = omp_get_wtime() - start_time;
        bool r_pos = b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos, 1);
        bool r_ori = b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori, 1);
        if(!r_pos || !r_ori)continue;
        
        curr_config.set_pos(pioneer_pos[0], pioneer_pos[1]);
        curr_config.set_theta(pioneer_ori[2]);
        W.update_config(curr_config);

        if(time > 0.3 && W.check_collision())
        {
            std::cout << "Colisão detectada!\n";
            break;
        }

        pioneer_model(0.2, -0.1, w_r, w_l);

        // //movimento circular
        // if(time <= 30.0){
        //     pioneer_model(0.0, 2.0*M_PI/30.0, w_r, w_l);
        // }else//movimento linear
        // {
        //     pioneer_model(0.2, 0.1, w_r, w_l);
        // }
        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor,  w_l, client.simxServiceCall());
    }while(time <= STOP_TIME);

    client.simxSetJointTargetVelocity(rightMotor, 0.0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor,  0.0, client.simxServiceCall());
    is_finished = true;
    
    std::cout << "Programa encerrando...\n";
    thr_plot.join();
    std::cout << "Programa encerrado!\n";
    client.simxStopSimulation(client.simxServiceCall());
    return 0;
}

void _init(int argc, char **argv)
{
    if(argc == 3)
    {
        SAMPLES = std::atoi(argv[1]);
        n_vertices = std::atoi(argv[2]);

        if((n_vertices % 2 != 0) || (n_vertices < 4)){
            std::cerr << "Número de vertices deve ser uma potência de 2 a partir de 4\n";
        }

        robot.set_shape( Polygon2D::circle_to_polygon2D(pioneer_radius, n_vertices));
        W.set_robot(robot);
        printf("Theta com %d amostras e Robo aproximado por poligono com %d vertices\n", SAMPLES, n_vertices);
    }else{
        printf("Argumentos exigidos: quantidade de pontos em theta e numero de vertices para representar o robo\n");
        printf("exemplo\n./p2m1 200 8\n");
    }
    Polygon2D obstacle;

    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if (!r)std::cerr << "Falha ao carregar o cenário!\n";
    std::string scene = string(cCurrentPath) + string("/scenes/p2m1_scene.ttt");
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());

    //Criando obstaculos
    {
        // Parede inferior
        obstacle = Polygon2D::rectangle_to_polygon2D(10.0, 0.1);
        obstacle = obstacle.translate(0.0, -5.0);
        W.add_obstacle(obstacle);
        // Parede da direita
        obstacle = Polygon2D::rectangle_to_polygon2D(10.0, 0.1);
        obstacle = obstacle.rotation(M_PI/2.0);
        obstacle = obstacle.translate(5.0,0.0);
        W.add_obstacle(obstacle);
        // Parede superior
        obstacle = Polygon2D::rectangle_to_polygon2D(10.0, 0.1);
        obstacle = obstacle.translate(0.0, 5.0);
        W.add_obstacle(obstacle);
        // Parede da esquerda
        obstacle = Polygon2D::rectangle_to_polygon2D(10.0, 0.1);
        obstacle = obstacle.rotation(M_PI/2.0);
        obstacle = obstacle.translate(-5.0,0.0);
        W.add_obstacle(obstacle);
        // Retangulo da esquerda
        obstacle = Polygon2D::rectangle_to_polygon2D(2.0, 0.5);
        obstacle = obstacle.translate(-4.0,-0.5);
        W.add_obstacle(obstacle);
        // Retangulo da direita
        obstacle = Polygon2D::rectangle_to_polygon2D(2.0, 0.5);
        obstacle = obstacle.translate(4.0,-0.5);
        W.add_obstacle(obstacle);
        // Retangulo central
        obstacle = Polygon2D::rectangle_to_polygon2D(3.0, 2.0);
        obstacle = obstacle.translate(0.0,-3.7);
        W.add_obstacle(obstacle);
        // Retangulo maior
        obstacle = Polygon2D::rectangle_to_polygon2D(5.0, 3.0);
        obstacle = obstacle.translate(0.0,2.75);
        W.add_obstacle(obstacle);
        // Octogono
        obstacle = Polygon2D::circle_to_polygon2D(1.0,8);
        obstacle = obstacle.translate(0.0,-0.5);
        W.add_obstacle(obstacle);
    }
    double tik,tok;
    tik = omp_get_wtime();
    W.compute_c_obstacles(SAMPLES);
    tok = omp_get_wtime();
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";

    printf("Tempo gasto calcular o espaço de configuração com %d amostras de theta  e %ld obstaculos | dt = %lf s\n", SAMPLES, W.get_obstacles().size(), tok - tik);
}

void living_plot()
{
    std::list<Polygon2D> obstacles = W.get_obstacles();
    std::list<Polygon2D> cb_obstacles;
    std::vector<double> x_vec, y_vec, z_vec, x_robot, y_robot, z_robot;
    std::vector<Config> config_hist;
    std::vector<std::tuple<
        std::vector<double>,
        std::vector<double>>>
        pts_obs, pts;
    Gnuplot gp_work;
    Gnuplot gp_curr_config;
    auto plots_work = gp_work.plotGroup();
    auto plots_curr_config = gp_curr_config.plotGroup();

    gp_work << "set title 'Espaço de trabalho'\n";
    gp_work << "load 'config_plot2D'\n";

    gp_curr_config << "set title 'Espaço de Configuração para o Theta Atual'\n";
    gp_curr_config << "load 'config_plot2D'\n";
    //ploting obstacles at work space
    {
        for (auto polygon_it = obstacles.begin(); polygon_it != obstacles.end(); polygon_it++)
        {
            x_vec.clear();
            y_vec.clear();
            Polygon2D::polygon_to_vectorsXY(*polygon_it, x_vec, y_vec);
            pts_obs.emplace_back(std::make_tuple(x_vec, y_vec));
        }
    }
    
    while (is_finished == false)
    {
        auto plots_work = gp_work.plotGroup();
        auto plots_curr_config = gp_curr_config.plotGroup();

        //objs
        plots_work.add_plot2d(pts_obs, "with filledcurve fc 'black'");

        // plot robot trajectory
        config_hist.push_back(W.get_robot().get_config());
        x_robot.push_back(W.get_robot().get_config().get_pos().x());
        y_robot.push_back(W.get_robot().get_config().get_pos().y());
        pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_work.add_plot2d(pts, "with line lc 'red' lw 1");

        //plot current position only
        x_vec.clear();
        y_vec.clear();
        Polygon2D::polygon_to_vectorsXY(W.get_robot().to_polygon2D(), x_vec, y_vec);
        pts.clear();
        pts.emplace_back(std::make_tuple(x_vec, y_vec));
        plots_work.add_plot2d(pts, "with line lc'red'");

        //plot config space
        pts.clear();
        cb_obstacles = W.get_cobstacles(SAMPLES);
        for (auto cb_it = cb_obstacles.begin(); cb_it != cb_obstacles.end(); cb_it++)
        {
            x_vec.clear();
            y_vec.clear();
            Polygon2D::polygon_to_vectorsXY(*cb_it, x_vec, y_vec);
            pts.emplace_back(std::make_tuple(x_vec, y_vec));
        }
        plots_curr_config.add_plot2d(pts, "with filledcurve fc 'black'");

        // robot at config space
        pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_curr_config.add_plot2d(pts, "with linespoints lc 'red' lt 7 lw 1");

        //show plot
        gp_work << plots_work;
        gp_curr_config << plots_curr_config;
    }


    // ploting the 3D config space
    #ifdef PLOT_CONFIG_SPACE
    Gnuplot gp_config;
    auto plots_config = gp_config.splotGroup();
    std::vector<std::tuple< std::vector<double>, std::vector<double>, std::vector<double> >> pts_config;
    gp_config << "load 'config_plot3D'\n";

    x_robot.clear();y_robot.clear();z_robot.clear();
    x_vec.clear();y_vec.clear();z_vec.clear();
    for(int i = 0; i < SAMPLES; i++)
    {   
        double th = (2.0*M_PI/SAMPLES)*i;
        W.update_config(Config(Vector2D(), th));
        cb_obstacles = W.get_cobstacles();
        
        for(auto cb_it = cb_obstacles.begin(); cb_it != cb_obstacles.end(); cb_it++)
        {   
            x_vec.clear();y_vec.clear();
            Polygon2D::polygon_to_vectorsXY(*cb_it, x_vec, y_vec);
            z_vec = std::vector<double>(x_vec.size(), th);
            pts_config.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
        }
        plots_config.add_plot2d(pts_config, "with polygons fc 'black'");
        // x_robot.push_back(config_hist[i].get_pos().x());
        // y_robot.push_back(config_hist[i].get_pos().y());
        // z_robot.push_back(config_hist[i].get_theta());
    }
    // pts_config.clear();
    // pts_config.emplace_back(std::make_tuple(x_robot,y_robot,z_robot));
    // plots_config.add_plot2d(pts_config, "with lines");
    gp_config << plots_config;
    #endif

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
}
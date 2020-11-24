
#include <gnuplot-iostream.h>        //Gnuplot
#include <configSpaceTools.hpp>      //Polygon2D, Robot, World, Vector2D, Config
#include <b0RemoteApi.h>             //para comunicação com o coppeliaSim via Blue-zero
#include <potentialField.hpp>        //PotentialField
#include <utils.hpp>                 //pioneer_model
#include <kinematicsControllers.hpp> //PathFollowController

#include <iostream> //cout
#include <stdio.h>  //printf
#include <cstdio>   //FILENAME_MAX, printf
#include <unistd.h> //getcwd
#include <string>   //string
#include <thread>   //thread
#include <omp.h>    //omp_get_wtime()
#include <list>     //list

using namespace std;

unsigned int SAMPLES = 200;
double pioneer_radius = 0.52 / 2.0;
unsigned int n_vertices = 8;
#define GetCurrentDir getcwd
#define STOP_TIME 9999 //segundos

void living_plot();
void _init(int argc, char **argv);

b0RemoteApi client("b0RemoteApi_CoppeliaSim-addOn", "b0RemoteApiAddOn");
Robot robot(Config(-3.0, -2.0, M_PI / 4.0), Polygon2D::circle_to_polygon2D(pioneer_radius, n_vertices));
World W(robot);
Config target_conf;
PotentialField pfield;
char cCurrentPath[FILENAME_MAX];
bool is_finished = false;

double v = 0.1;
double K_lin = 0.5;
double K_ang = 1.6;
PathFollowController controller(K_ang, K_lin);
std::list<Config> path;

int main(int argc, char **argv)
{
    Config curr_config, ref;
    std::thread thr_plot;
    std::vector<float> pioneer_pos, pioneer_ori;
    std::vector<float> target_pos, target_ori;
    int pioneer, leftMotor, rightMotor, target;
    float w_r, w_l;
    double w, lin_error, ang_error;

    target = b0RemoteApi::readInt(client.simxGetObjectHandle("Target", client.simxServiceCall()), 1);
    pioneer = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx", client.simxServiceCall()), 1);
    leftMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_leftMotor", client.simxServiceCall()), 1);
    rightMotor = b0RemoteApi::readInt(client.simxGetObjectHandle("Pioneer_p3dx_rightMotor", client.simxServiceCall()), 1);

    //obtendo configuração do target
    bool t_pos = b0RemoteApi::readFloatArray(client.simxGetObjectPosition(target, -1, client.simxServiceCall()), target_pos, 1);
    bool t_ori = b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(target, -1, client.simxServiceCall()), target_ori, 1);
    target_conf = Config(target_pos[0], target_pos[1], target_ori[2]);

    bool r_pos = b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos, 1);
    bool r_ori = b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori, 1);
    curr_config.set_pos(pioneer_pos[0], pioneer_pos[1]);
    curr_config.set_theta(pioneer_ori[2]);
    W.update_config(curr_config);
    _init(argc, argv);

    thr_plot = std::thread(living_plot);
    double time = 0, start_time = omp_get_wtime();
    do
    {
        time = omp_get_wtime() - start_time;
        r_pos = b0RemoteApi::readFloatArray(client.simxGetObjectPosition(pioneer, -1, client.simxServiceCall()), pioneer_pos, 1);
        r_ori = b0RemoteApi::readFloatArray(client.simxGetObjectOrientation(pioneer, -1, client.simxServiceCall()), pioneer_ori, 1);
        if (!r_pos || !r_ori)
        {
            std::cout << "Falha ao receber dados do simulador!\n";
            continue;
        }

        curr_config.set_pos(pioneer_pos[0], pioneer_pos[1]);
        curr_config.set_theta(pioneer_ori[2]);
        W.update_config(curr_config);

        if (time > 0.3 && W.check_collision())
        {
            std::cout << "Colisão detectada!\n";
            break;
        }

        is_finished = controller.step(curr_config, v, w, ref, lin_error, ang_error);
        pioneer_model(0.2, w, w_r, w_l);

        client.simxSetJointTargetVelocity(rightMotor, w_r, client.simxServiceCall());
        client.simxSetJointTargetVelocity(leftMotor, w_l, client.simxServiceCall());

        std::cout << "REF = " << ref << " | ";
        printf("Angular Error = %lf | Linear Error = %lf \n", ang_error, lin_error);
        // client.simxSetJointTargetVelocity(rightMotor, 0.0, client.simxServiceCall());
        // client.simxSetJointTargetVelocity(leftMotor, 0.0, client.simxServiceCall());
    } while ((time <= STOP_TIME) && (is_finished == false));

    client.simxSetJointTargetVelocity(rightMotor, 0.0, client.simxServiceCall());
    client.simxSetJointTargetVelocity(leftMotor, 0.0, client.simxServiceCall());
    is_finished = true;

    std::cout << "Programa encerrando...\n";
    thr_plot.join();
    std::cout << "Programa encerrado!\n";
    client.simxStopSimulation(client.simxServiceCall());
    return 0;
}

void _init(int argc, char **argv)
{
    if (argc == 3)
    {
        K_ang = std::atof(argv[1]);
        K_lin = std::atof(argv[2]);
        printf("Ganho angular = %lf | Ganho linear = %lf\n", K_ang, K_lin);
    }
    else
    {
        std::cerr << "Voce pode passar os ganhos do controlador por linha de comando: K_ang K_lin\n";
        // printf("Argumentos exigidos: quantidade de pontos em theta e numero de vertices para representar o robo\n");
        // printf("exemplo\n./p2m1 200 8\n");
    }
    Polygon2D obstacle;

    /*
    bool r = GetCurrentDir(cCurrentPath, sizeof(cCurrentPath));
    if (!r)
        std::cerr << "Falha ao carregar o cenário!\n";
    std::string scene = string(cCurrentPath) + string("/scenes/p2m1_scene.ttt");
    client.simxLoadScene(scene.c_str(), client.simxServiceCall());
    */

    //Criando obstaculos
    {   
        /* //Exemplo 1
        // Octogono
        obstacle = Polygon2D::circle_to_polygon2D(1.0, 8);
        obstacle = obstacle.translate(-5.7500e-01, -1.7500e-01);
        W.add_obstacle(obstacle);
        */

        //Exemplo 2
        /*
        obstacle = Polygon2D::circle_to_polygon2D(1.0, 8);
        obstacle = obstacle.translate(-1.1500e+00, +1.4000e+00);
        W.add_obstacle(obstacle);

        obstacle = Polygon2D::circle_to_polygon2D(1.0, 8);
        obstacle = obstacle.translate(+9.2500e-01,+4.9999e-02);
        W.add_obstacle(obstacle);
        */

       //Exemplo 3
        obstacle = Polygon2D::circle_to_polygon2D(1.0, 8);
        obstacle = obstacle.translate(+4.9998e-02, +3.2500e-01);
        W.add_obstacle(obstacle);

        // obstacle = Polygon2D::circle_to_polygon2D(1.0, 8);
        // obstacle = obstacle.translate(+3.1000e+00,+9.7500e-01);
        // W.add_obstacle(obstacle);

         

        // obstacle = Polygon2D::circle_to_polygon2D(1.0, 8);
        // obstacle = obstacle.translate(-3.0750e+00, -2.2500e-01);
        // W.add_obstacle(obstacle);
    }
    double tik, tok;
    tik = omp_get_wtime();
    W.compute_c_obstacles(SAMPLES);
    tok = omp_get_wtime();
    client.simxStartSimulation(client.simxServiceCall());
    std::cout << "Conectado!\n";

    printf("Tempo gasto calcular o espaço de configuração com %d amostras de theta  e %ld obstaculos | dt = %lf s\n", SAMPLES, W.get_obstacles().size(), tok - tik);

    //Planejamento de rota
    pfield.set_space(W);
    std::cout << "Computando rota...\n";
    tik = omp_get_wtime();
    path = pfield.get_planned_path(W.get_robot().get_config(), target_conf, pioneer_radius, 0.001);
    tok = omp_get_wtime();
    std::cout << "Tempo gasto para computar a rota:" << tok - tik << "s\n";
    std::cout << "Numero de pontos:" << path.size() << '\n';

    controller.update(K_ang, K_lin, path);
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
        pts_obs, pts, pts_path;
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
        //ploting the planned plath
        x_vec.clear();
        y_vec.clear();
        for (auto p = path.begin(); p != path.end(); p++)
        {
            x_vec.push_back(p->x());
            y_vec.push_back(p->y());
        }
        pts_path.emplace_back(std::make_tuple(x_vec, y_vec));
    }

    while (is_finished == false)
    {
        auto plots_work = gp_work.plotGroup();
        auto plots_curr_config = gp_curr_config.plotGroup();

        //objs
        plots_work.add_plot2d(pts_obs, "with filledcurve fc 'black'");
        plots_work.add_plot2d(pts_path, "with line lc 'blue' lt 23");

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

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
}
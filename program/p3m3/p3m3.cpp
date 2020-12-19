#include "p3m3.hpp"

#include <gnuplot-iostream.h> //Gnuplot

#include <omp.h> //omp_get_wtime()

void _receiver_func(void *X)
{
    static_cast<Simulation_p3m3 *>(X)->_receiver_routine();
};
void _plotter_func(void *X)
{
    static_cast<Simulation_p3m3 *>(X)->_plotter_routine();
};

Simulation_p3m3::Simulation_p3m3() : thr_plotter(),
                                     thr_receiver(),
                                     mtx(),
                                     client(nullptr),
                                     time_to_stop(0.0),
                                     to_stop(false),
                                     handle_pioneer(),
                                     handle_leftMotor(),
                                     handle_rightMotor(),
                                     myqueue(),
                                     my_prob_occup_grid(20, 20, 0.1),
                                     my_regular_grid(Vector2D(0, 0), 20, 20, 0.1),
                                     my_path_cell(),
                                     my_path_follower()
{   
    my_robot = Robot(Config(0, 0, 0), Polygon2D::rectangle_to_polygon2D(5.1900e-01, 4.1500e-01));
    mtx.unlock();
}

Simulation_p3m3::~Simulation_p3m3()
{
    this->stop_simulation();
}

void Simulation_p3m3::stop_simulation()
{
    this->to_stop = true;
    if (thr_receiver.joinable())
        thr_receiver.join();

    if (thr_plotter.joinable())
        thr_plotter.join();

    if (client != nullptr)
    {
        client->simxStopSimulation(client->simxServiceCall());
    }
    // my_occup_grid.save_to_file("occupating_grid.save");
}

void Simulation_p3m3::start_simulation(const std::string &scene, const float &time_to_stop)
{
    std::vector<float> target_pos;

    my_prob_occup_grid.load_from_file("occupating_grid.save");

    this->stop_simulation();
    this->to_stop = false;

    client = new b0RemoteApi(nodeName.c_str(), channelName.c_str());
    this->time_to_stop = time_to_stop;

    if (!scene.empty())
        client->simxLoadScene(scene.c_str(), client->simxServiceCall());

    client->simxStartSimulation(client->simxServiceCall());

    handle_pioneer = b0RemoteApi::readInt(client->simxGetObjectHandle("Pioneer_p3dx", client->simxServiceCall()), 1);
    handle_leftMotor = b0RemoteApi::readInt(client->simxGetObjectHandle("Pioneer_p3dx_leftMotor", client->simxServiceCall()), 1);
    handle_rightMotor = b0RemoteApi::readInt(client->simxGetObjectHandle("Pioneer_p3dx_rightMotor", client->simxServiceCall()), 1);
    handle_target = b0RemoteApi::readInt(client->simxGetObjectHandle("Target", client->simxServiceCall()), 1);

    _readDatas();

    b0RemoteApi::readFloatArray(client->simxGetObjectPosition(handle_target, -1, client->simxServiceCall()), target_pos, 1);

    my_regular_grid = manhattan( my_prob_occup_grid, Config(target_pos[0], target_pos[1], 0),  my_robot, occupating_thr);
    my_path_cell = bestFirst( my_regular_grid, myqueue.front().q, Config(target_pos[0], target_pos[1], 0));
    
    if(my_path_cell.empty())
    {
        std::cerr << "Falha ao encontrar um caminho\n";
        to_stop = true;
    }else{
        std::list<Config> points;
        for(auto cell : my_path_cell)
            points.push_back( Config(cell.pos,0) );
        my_path_follower.update(K_ang, K_lin, points);
    }

    thr_receiver = std::thread(_receiver_func, this);
    assert(thr_receiver.joinable());

    thr_plotter = std::thread(_plotter_func, this);
    assert(thr_plotter.joinable());
}
void Simulation_p3m3::_receiver_routine()
{
    double tik, tok;

    tik = omp_get_wtime();
    tok = omp_get_wtime();
    while (to_stop == false)
    {
        if ((tok - tik) >= time_to_stop)
        {
            std::cerr << "Time out!\n";
            to_stop = true;
            continue;
        }
        _readDatas();
        tok = omp_get_wtime();
    }

    std::cout << "Receiver parou!\n";
}

void Simulation_p3m3::_readDatas()
{
    static MyDatas datas;
    static bool r_pos, r_ori;
    static std::vector<float> pos({0, 0}), ori({0, 0, 0});
    static std::vector<float> target_pos({0, 0});

    r_pos = b0RemoteApi::readFloatArray(client->simxGetObjectPosition(handle_pioneer, -1, client->simxServiceCall()), pos, 1);
    r_ori = b0RemoteApi::readFloatArray(client->simxGetObjectOrientation(handle_pioneer, -1, client->simxServiceCall()), ori, 1);

    if (!r_pos || !r_ori)
    {
        std::cerr << "Falha no recebimento dos dados!\n";
        return;
    }
    //atualiza a configuração atual
    datas.q.set_pos(pos[0], pos[1]);
    datas.q.set_theta(ori[2]);
    //coloca na fila compartilhada
    mtx.lock(); //LOCK
    myqueue.push(datas);
    mtx.unlock(); //UNLOCK
}

void Simulation_p3m3::_plotter_routine()
{
    std::cout << "Plotter outine\n";
    std::vector<double> x_robot, y_robot, z_robot, x_vec, y_vec, z_vec;
    std::vector<double> sensor_point_x, sensor_point_y;
    std::vector<Config> config_hist;
    std::vector<std::tuple<std::vector<double>, std::vector<double>>> pts;
    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> pts3D;
    Gnuplot gp_manhattan, gp_OG_prob, gp_OG, gp_work;
    MyDatas datas;
    std::vector<OccupationGridCell> prob_og;

    auto plots_manhattan = gp_manhattan.plotGroup();
    auto plots_OG_prob = gp_OG_prob.plotGroup();
    auto plots_OG = gp_OG.plotGroup();

    gp_work << "set title 'Espaço de Trabalho'\n";
    gp_work << "load 'config_plot2D'\n";

    gp_OG << "set title 'Grade de Ocupação'\n";
    gp_OG << "load 'config_plot2D'\n";
    gp_OG << "unset colorbox\n";
    gp_OG << "set pm3d map\n";
    gp_OG << "set palette defined (0 'white', 1 'black')\n";

    gp_OG_prob << "set title 'Grade de Ocupação Probabilística'\n";
    gp_OG_prob << "load 'config_plot2D'\n";
    gp_OG_prob << "set pm3d \n";
    gp_OG_prob << "set dgrid3d\n";

    gp_manhattan << "set title 'Campo Potencial'\n";
    gp_manhattan << "load 'config_plot2D'\n";
    gp_manhattan << "set pm3d \n";
    gp_manhattan << "set dgrid3d\n";

    //plotting probabilistic occupation grid
    prob_og = my_prob_occup_grid.get_OG();
    x_vec.clear();
    y_vec.clear();
    z_vec.clear();
    pts3D.clear();
    for (std::size_t i = 0; i < prob_og.size(); i++)
    {
        x_vec.push_back(prob_og[i].pos.x());
        y_vec.push_back(prob_og[i].pos.y());
        z_vec.push_back(prob_og[i].p);
    }
    pts3D.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
    plots_OG_prob.add_plot2d(pts3D, "with image");
    gp_OG_prob << plots_OG_prob;

    //plotting manhattan function result
    z_vec.clear();
    pts3D.clear();
    for (std::size_t i = 0; i < my_regular_grid.size(); i++)
    {   
        if(abs(my_regular_grid[i].value) == M)
            z_vec.push_back(0);
        else
            z_vec.push_back(my_regular_grid[i].value);
    }
    pts3D.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
    plots_manhattan.add_plot2d(pts3D, "with image");
    gp_manhattan << plots_manhattan;

    //plotting occupation grid
    z_vec.clear();
    pts3D.clear();
    for (std::size_t i = 0; i < my_regular_grid.size(); i++)
    {
        z_vec.push_back(my_regular_grid[i].free);
    }
    pts3D.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
    plots_OG.add_plot2d(pts3D, "with image");
    gp_OG << plots_OG;

    while (to_stop == false)
    {
        auto plots_work = gp_work.plotGroup();
        //Atualiza dados
        {
            mtx.lock();
            while (myqueue.empty()) //sem dados novos na fila
            {
                mtx.unlock();
                std::this_thread::yield();
                mtx.lock();
            }
            datas = myqueue.front(); //novos dados
            myqueue.pop();
            mtx.unlock();
        }

        //atualizaz a configuracao do robo
        my_robot.set_config(datas.q);

        // plot robot trajectory
        config_hist.push_back(datas.q);
        x_robot.push_back(datas.q.x());
        y_robot.push_back(datas.q.y());
        pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_work.add_plot2d(pts, "with line lc 'red' lw 1");

        //plot path
        x_vec.clear();
        y_vec.clear();
        for (auto cell : my_path_cell)
        {
            x_vec.push_back(cell.pos.x());
            y_vec.push_back(cell.pos.y());
        }
        pts.clear();
        pts.emplace_back(std::make_tuple(x_vec, y_vec));
        plots_work.add_plot2d(pts, "with line lc 'blue' lw 1");

        //plot current position only
        x_vec.clear();
        y_vec.clear();
        Polygon2D::polygon_to_vectorsXY(my_robot.to_polygon2D(), x_vec, y_vec);
        pts.clear();
        pts.emplace_back(std::make_tuple(x_vec, y_vec));
        plots_work.add_plot2d(pts, "with line lc'red'");

        //show plot
        gp_work << plots_work;
    }

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
}

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
                                     handle_sensor(),
                                     myqueue(),
                                     my_occup_grid(20, 20, 0.1)
{
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
        delete client;
    }
    // my_occup_grid.save_to_file("occupating_grid.save");
}

void Simulation_p3m3::start_simulation(const std::string &scene, const float &time_to_stop)
{
    my_occup_grid.load_from_file("occupating_grid.save");
    
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

    std::string tmp_str;
    for (int i = 0; i < N_SENSORS; i++)
    {
        tmp_str = std::string("Sensor") + std::to_string(i);
        handle_sensor[i] = b0RemoteApi::readInt(client->simxGetObjectHandle(tmp_str.c_str(), client->simxServiceCall()), 1);
    }

    _readDatas();

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

void Simulation_p3m3::_plotter_routine()
{
    std::cout << "Plotter outine\n";
    std::vector<double> x_robot, y_robot, z_robot, x_vec, y_vec, z_vec;
    std::vector<double> sensor_point_x, sensor_point_y;
    std::vector<Config> config_hist;
    std::vector<std::tuple<std::vector<double>, std::vector<double>>> pts;
    std::vector<std::tuple<std::vector<double>, std::vector<double>,std::vector<double>>> pts3D;
    Gnuplot gp_work, gp_OG_prob, gp_OG_bin;
    MyDatas datas;
    Robot robot(Config(0, 0, 0), Polygon2D::rectangle_to_polygon2D(5.1900e-01, 4.1500e-01));
    std::vector<CellGrid> og;

    auto plots_work = gp_work.plotGroup();
    auto plots_OG   = gp_OG_prob.plotGroup();

    gp_work << "set title 'Espaço de trabalho'\n";
    gp_work << "load 'config_plot2D'\n";

    gp_OG_prob   << "load 'config_plot2D'\n";
    gp_OG_prob   << "set pm3d \n";
    gp_OG_prob   << "set dgrid3d\n";

    gp_OG_bin   << "load 'config_plot2D'\n";
    gp_OG_bin   << "unset colorbox\n";
    gp_OG_bin   << "set pm3d map\n";
    gp_OG_bin   << "set palette defined (0 'white', 1 'black')\n";
    
    //plotting occupation grid
    og = my_occup_grid.get_OG();
    x_vec.clear();
    y_vec.clear();
    z_vec.clear();
    pts3D.clear();
    for(std::size_t i = 0; i < og.size(); i++)
    {
        x_vec.push_back( og[i].pos.x() );
        y_vec.push_back( og[i].pos.y() );
        z_vec.push_back( og[i].p);
    }
    pts3D.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
    plots_OG.add_plot2d(pts3D, "with image");
    gp_OG_prob   << plots_OG;

    while (to_stop == false)
    {
        auto plots_work = gp_work.plotGroup();
        auto plots_OG   = gp_OG_bin.plotGroup();

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

        //atualizaz a configuracao do robo
        robot.set_config(datas.q);

        // // plot robot trajectory
        config_hist.push_back(datas.q);
        x_robot.push_back(datas.q.x());
        y_robot.push_back(datas.q.y());
        pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_work.add_plot2d(pts, "with line lc 'red' lw 1");

        //plot current position only
        x_vec.clear();
        y_vec.clear();
        Polygon2D::polygon_to_vectorsXY(robot.to_polygon2D(), x_vec, y_vec);
        pts.clear();
        pts.emplace_back(std::make_tuple(x_vec, y_vec));
        plots_work.add_plot2d(pts, "with line lc'red'");

        // for (int is = 0; is < N_SENSORS; is++)
        // {
        //     if (datas.sensors[is].zt() > 0)
        //     {
        //         sensor_point_x.push_back(datas.sensors[is].point().x());
        //         sensor_point_y.push_back(datas.sensors[is].point().y());
        //     }
        // }
        // if (!sensor_point_x.empty())
        // {
        //     pts.clear();
        //     pts.emplace_back(std::make_tuple(sensor_point_x, sensor_point_y));
        //     plots_work.add_plot2d(pts, "with points");
        // }

        //plotting occupation grid
        og = my_occup_grid.get_OG();
        x_vec.clear();
        y_vec.clear();
        z_vec.clear();
        pts3D.clear();
        for(std::size_t i = 0; i < og.size(); i++)
        {
            x_vec.push_back( og[i].pos.x() );
            y_vec.push_back( og[i].pos.y() );
            z_vec.push_back( (og[i].p < occupating_thr)?0:1);
        }
        pts3D.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
        plots_OG.add_plot2d(pts3D, "with image");
        gp_OG_bin   << plots_OG;

        //show plot
        gp_work << plots_work;
        gp_OG_bin << plots_OG;
    }

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
}

void Simulation_p3m3::_readDatas()
{
    static MyDatas datas;
    static bool r_sensor, r_pos, r_ori;
    static std::vector<float> pos({0, 0}), ori({0, 0, 0}), sensor_pt;
    static std::vector<msgpack::v2::object> *sensor_return;
    static int sensor_state;
    static float measure_dist;

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

    //atualiza as leituras dos sensores de proximidade
    for (int is = 0; is < N_SENSORS; is++)
    {
        sensor_return = client->simxReadProximitySensor(handle_sensor[is], client->simxServiceCall());

        r_sensor = b0RemoteApi::readBool(sensor_return, 0);

        if (!r_sensor)
        {
            std::cerr << "Falha no recebimento dos dados!\n";
            continue;
        }
        sensor_state = b0RemoteApi::readInt(sensor_return, 1);
        if (sensor_state == 1) //se ha dado do sensor
        {
            measure_dist = b0RemoteApi::readFloat(sensor_return, 2);
            b0RemoteApi::readFloatArray(sensor_return, sensor_pt, 3);
            datas.sensors[is].updateMeasure(measure_dist, Vector2D(sensor_pt[2], sensor_pt[0]));
        }else{
            datas.sensors[is].updateMeasure(-1, Vector2D());
        }
        my_occup_grid.update(datas.sensors[is], datas.q);
    }
    _sensor_to_world_frame(datas);
    //coloca na fila compartilhada
    mtx.lock(); //LOCK
    myqueue.push(datas);
    mtx.unlock(); //UNLOCK
}

void Simulation_p3m3::_sensor_to_world_frame(MyDatas &datas)
{
    for (int is = 0; is < N_SENSORS; is++)
    {
        if (datas.sensors[is].zt() < 0)
            continue;
        //coordenadas do sensor para coordenadas do robo
        datas.sensors[is].point() = datas.sensors[is].point().rotation(M_PI_2 * is);
        datas.sensors[is].point() = datas.sensors[is].point() + Vector2D(0.2, 0);

        //coordenadas do robo para coordenadas do mundo
        datas.sensors[is].point() = datas.sensors[is].point().rotation(datas.q.theta());
        datas.sensors[is].point() = datas.sensors[is].point() + datas.q.get_pos();
    }
}

#include "p3m1.hpp"

#include <gnuplot-iostream.h> //Gnuplot

#include <omp.h>  //omp_get_wtime()

void _receiver_func(void *X)
{
    static_cast<Simulation_p3m1 *>(X)->_receiver_routine();
};
void _plotter_func(void *X)
{
    static_cast<Simulation_p3m1 *>(X)->_plotter_routine();
};

Simulation_p3m1::Simulation_p3m1() : thr_plotter(),
                                     thr_receiver(),
                                     mtx_currConfig(),
                                     client(nullptr),
                                     currConfig(0, 0, 0),
                                     time_to_stop(0.0),
                                     to_stop(false),
                                     handle_pioneer(),
                                     handle_leftMotor(),
                                     handle_rightMotor(),
                                     handle_sensor(),
                                     stab_control(0, 0, 0, 0, 0, 0)
{
    mtx_currConfig.unlock();
}

Simulation_p3m1::~Simulation_p3m1()
{
    if (thr_receiver.joinable())
        thr_receiver.join();
    if (thr_plotter.joinable())
        thr_plotter.join();

    if (client != nullptr)
    {
        client->simxStopSimulation(client->simxServiceCall());
        delete client;
    }
    to_stop = false;
    stab_control.reset();
}

void Simulation_p3m1::start_simulation(const std::string &scene, const float &time_to_stop)
{
    client = new b0RemoteApi(nodeName.c_str(), channelName.c_str());
    if (!scene.empty())
        client->simxLoadScene(scene.c_str(), client->simxServiceCall());

    client->simxStartSimulation(client->simxServiceCall());

    handle_pioneer = b0RemoteApi::readInt(client->simxGetObjectHandle("Pioneer_p3dx", client->simxServiceCall()), 1);
    handle_leftMotor = b0RemoteApi::readInt(client->simxGetObjectHandle("Pioneer_p3dx_leftMotor", client->simxServiceCall()), 1);
    handle_rightMotor = b0RemoteApi::readInt(client->simxGetObjectHandle("Pioneer_p3dx_rightMotor", client->simxServiceCall()), 1);
    handle_target = b0RemoteApi::readInt(client->simxGetObjectHandle("Target", client->simxServiceCall()), 1);
    
    handle_sensor = b0RemoteApi::readInt(client->simxGetObjectHandle("Sensor0", client->simxServiceCall()), 1);

    std::vector<float> pos({0, 0}), ori({0, 0, 0});
    b0RemoteApi::readFloatArray(client->simxGetObjectPosition(handle_pioneer, -1, client->simxServiceCall()), pos, 1);
    b0RemoteApi::readFloatArray(client->simxGetObjectOrientation(handle_pioneer, -1, client->simxServiceCall()), ori, 1);
    currConfig.set_pos(pos[0], pos[1]);
    currConfig.set_theta(ori[2]);

    this->time_to_stop = time_to_stop;

    thr_receiver = std::thread(_receiver_func, this);
    assert(thr_receiver.joinable());

    thr_plotter = std::thread(_plotter_func, this);
    assert(thr_plotter.joinable());

    while(to_stop == false)
    {
        std::this_thread::yield();
    }
}
void Simulation_p3m1::_receiver_routine()
{
    std::vector<float> pos({0, 0}), ori({0, 0, 0}), sensor_pt;
    std::vector<float> target_pos({0, 0}), target_ori({0, 0, 0});
    Config q_ref;
    bool r_pos, r_ori, t_pos, t_ori;
    bool r_control;
    double tik,tok;
    float w_right, w_left, v, w;

    stab_control.update(Kp_lin, Ki_lin, Kd_lin,
                        Kp_ang, Ki_ang, Kd_ang);
    tik = omp_get_wtime();
    tok = omp_get_wtime();
    while (to_stop == false)
    {
        if((tok - tik) >= time_to_stop)
        {
            std::cerr << "Time out!\n";
            to_stop = true;
            continue;
        }

        r_pos = b0RemoteApi::readFloatArray(client->simxGetObjectPosition(handle_pioneer, -1, client->simxServiceCall()), pos, 1);
        r_ori = b0RemoteApi::readFloatArray(client->simxGetObjectOrientation(handle_pioneer, -1, client->simxServiceCall()), ori, 1);
        t_pos = b0RemoteApi::readFloatArray(client->simxGetObjectPosition(handle_target, -1, client->simxServiceCall()), target_pos, 1);
        t_ori = b0RemoteApi::readFloatArray(client->simxGetObjectOrientation(handle_target, -1, client->simxServiceCall()), target_ori, 1);
        
        auto sensor_return = client->simxReadProximitySensor(handle_sensor, client->simxServiceCall());
        
        bool br = b0RemoteApi::readBool(sensor_return, 0);
        std::cout << "Bool:" << br << '\n';
        if(br)
        {   
            int ir = b0RemoteApi::readInt(sensor_return, 1);
            std::cout << "Int:" << ir << '\n';
            if(ir == 1)
            {   
                std::cout << "Colisao detectada!\n";
                std::cout << "Float:" << b0RemoteApi::readFloat(sensor_return, 2) << '\n';
                b0RemoteApi::readFloatArray(sensor_return, sensor_pt, 3);
                std::cout << "Point:(" << sensor_pt[0] <<','<< sensor_pt[1] <<','<< sensor_pt[1] << '\n';
            }
            else
                std::cout << "Nada detectado.\n";
        }else{
            std::cout << "Falha no recebimento\n";
        }
        
        if (!r_pos || !r_ori || !t_pos || !t_ori)
        {
            std::cerr << "Falha no recebimento de dados!\n";
            continue;
        }

        mtx_currConfig.lock();      //LOCK

        currConfig.set_pos(pos[0], pos[1]);
        currConfig.set_theta(ori[2]);
        q_ref = Config(target_pos[0], target_pos[1], target_ori[2]);

        r_control = stab_control.step(q_ref, currConfig);

        mtx_currConfig.unlock();   //UNLOCK

        v = stab_control.get_v();
        w = stab_control.get_w();
        

        // pioneer_model(v, w, w_right, w_left);
        // client->simxSetJointTargetVelocity(handle_rightMotor, w_right, client->simxServiceCall());
        // client->simxSetJointTargetVelocity(handle_leftMotor, w_left, client->simxServiceCall());

        tok = omp_get_wtime();
    }

    std::cout << "Receiver parou!\n";

}

void Simulation_p3m1::_plotter_routine()
{
    std::cout << "Plotter outine\n";
    std::vector<double> x_robot, y_robot, z_robot;
    std::vector<Config> config_hist;
    std::vector<std::tuple<
        std::vector<double>,
        std::vector<double>>>
        pts;
    Gnuplot gp_work;
    auto plots_work = gp_work.plotGroup();

    gp_work << "set title 'Espaço de trabalho'\n";
    gp_work << "load 'config_plot2D'\n";

    Config q;
    while (to_stop == false)
    {
        auto plots_work = gp_work.plotGroup();

        mtx_currConfig.lock();
        q = currConfig;
        mtx_currConfig.unlock();
        
        // plot robot trajectory
        config_hist.push_back( q );
        x_robot.push_back( q.x() );
        y_robot.push_back( q.y() );
        // pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_work.add_plot2d(pts, "with line lc 'red' lw 1");

        //show plot
        gp_work << plots_work;
    }

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
}
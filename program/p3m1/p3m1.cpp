#include "p3m1.hpp"

#include <gnuplot-iostream.h> //Gnuplot

#include <omp.h> //omp_get_wtime()

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
                                     mtx(),
                                     client(nullptr),
                                     time_to_stop(0.0),
                                     to_stop(false),
                                     handle_pioneer(),
                                     handle_leftMotor(),
                                     handle_rightMotor(),
                                     handle_sensor(),
                                     myqueue()
{
    mtx.unlock();
}

Simulation_p3m1::~Simulation_p3m1()
{
    this->stop_simulation();
}

void Simulation_p3m1::stop_simulation()
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
}

void Simulation_p3m1::start_simulation(const std::string &scene, const float &time_to_stop)
{
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
    for(int i = 0; i < N_SENSORS; i++)
    {
      tmp_str = std::string("Sensor")+std::to_string(i);
      handle_sensor[i] = b0RemoteApi::readInt(client->simxGetObjectHandle(tmp_str.c_str(), client->simxServiceCall()), 1);
    }

    _readDatas();

    thr_receiver = std::thread(_receiver_func, this);
    assert(thr_receiver.joinable());

    thr_plotter = std::thread(_plotter_func, this);
    assert(thr_plotter.joinable());
}
void Simulation_p3m1::_receiver_routine()
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

void Simulation_p3m1::_plotter_routine()
{
    std::cout << "Plotter outine\n";
    std::vector<double> x_robot, y_robot, z_robot, x_vec, y_vec;
    std::vector<double> sensor_point_x, sensor_point_y;
    std::vector<Config> config_hist;
    std::vector<std::tuple<std::vector<double>, std::vector<double>>> pts;
    Gnuplot gp_work;
    MyDatas datas;
    Robot robot(Config(0, 0, 0), Polygon2D::rectangle_to_polygon2D(5.1900e-01, 4.1500e-01));

    auto plots_work = gp_work.plotGroup();

    gp_work << "set title 'Espaço de trabalho'\n";
    gp_work << "load 'config_plot2D'\n";

    while (to_stop == false)
    {
        auto plots_work = gp_work.plotGroup();

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

        // plot robot trajectory
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

	    for(int is = 0; is < N_SENSORS; is++)
	    {
		    if(datas.sensor_state[is] == 1)
		    {
		        sensor_point_x.push_back(datas.sensor_measure_point[is].x());
		        sensor_point_y.push_back(datas.sensor_measure_point[is].y());
		    }
      }
	    if (!sensor_point_x.empty())
	    {
	        pts.clear();
	        pts.emplace_back(std::make_tuple(sensor_point_x, sensor_point_y));
	        plots_work.add_plot2d(pts, "with points");
	    }

        //show plot
        gp_work << plots_work;
    }

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
}

void Simulation_p3m1::_readDatas()
{
    static MyDatas datas;
    static bool r_sensor, r_pos, r_ori;
    static std::vector<float> pos({0, 0}), ori({0, 0, 0}), sensor_pt;
    static std::vector<msgpack::v2::object> *sensor_return;

    r_pos = b0RemoteApi::readFloatArray(client->simxGetObjectPosition(handle_pioneer, -1, client->simxServiceCall()), pos, 1);
    r_ori = b0RemoteApi::readFloatArray(client->simxGetObjectOrientation(handle_pioneer, -1, client->simxServiceCall()), ori, 1);

    if(!r_pos || !r_ori)
    {
      std::cerr << "Falha no recebimento dos dados!\n";
      return;
    }
    //atualiza a configuração atual
    datas.q.set_pos(pos[0], pos[1]);
    datas.q.set_theta(ori[2]);

    //atualiza as leituras dos sensores de proximidade
    for(int is = 0; is < N_SENSORS; is++)
    {
    	sensor_return = client->simxReadProximitySensor(handle_sensor[is], client->simxServiceCall());

    	r_sensor = b0RemoteApi::readBool(sensor_return, 0);

    	if (!r_sensor)
    	{
    		std::cerr << "Falha no recebimento dos dados!\n";
    		continue;
    	}
    	datas.sensor_state[is] = b0RemoteApi::readInt(sensor_return, 1);
    	if (datas.sensor_state[is] == 1) //se ha dado do sensor
    	{
    		datas.sensor_measure_dist[is] = b0RemoteApi::readFloat(sensor_return, 2);
    		b0RemoteApi::readFloatArray(sensor_return, sensor_pt, 3);
    		datas.sensor_measure_point[is] = Vector2D(sensor_pt[2], sensor_pt[0]);
    	}
    }
    _sensor_to_world_frame(datas);
    //coloca na fila compartilhada
    mtx.lock(); //LOCK
    myqueue.push(datas);
    mtx.unlock(); //UNLOCK
}

void Simulation_p3m1::_sensor_to_world_frame(MyDatas &datas)
{
    for(int is = 0; is < N_SENSORS; is++)
	  {
      if(datas.sensor_state[is] != 1)
        continue;
  		//coordenadas do sensor para coordenadas do robo
  		datas.sensor_measure_point[is] = datas.sensor_measure_point[is].rotation(M_PI_2*is);
  		datas.sensor_measure_point[is] = datas.sensor_measure_point[is] + Vector2D(0.2,0);

  		//coordenadas do robo para coordenadas do mundo
  		datas.sensor_measure_point[is] = datas.sensor_measure_point[is].rotation(datas.q.theta());
  		datas.sensor_measure_point[is] = datas.sensor_measure_point[is] + datas.q.get_pos();
	 }
}

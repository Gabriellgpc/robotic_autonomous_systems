#include <matplotlibcpp.h>
#include <gnuplot-iostream.h>

#include <configSpaceTools.hpp>
#include <iostream> //cout
#include <stdio.h>  //printf

#include <list>
#include <vector>
#include <cmath>
#include <string>
#include <thread>
#include <omp.h>

using namespace std;
namespace plt = matplotlibcpp;

void plot_config_space(World w, Gnuplot *gp = NULL, const unsigned int n_samples = 100);
void polygon_to_vectorsXY(const Polygon2D &polygon, std::vector<double> &vertices_x, std::vector<double> &vertices_y);

void living_plot();
void plot_path(std::vector<Config> config_hist, Gnuplot *gp);

#define SAMPLES 100

Robot robot(Config(), Polygon2D::circle_to_polygon2D(1.0, 8));
World W(robot);
bool is_finished = false;

int main(int argc, char **argv)
{
    Polygon2D polygon;
    double tik, tok;
    std::thread thr_plot;

    {
        polygon = Polygon2D::rectangle_to_polygon2D(Vector2D(-10, 0), 5.0, 5.0);
        W.add_obstacle(polygon);
        polygon = Polygon2D::rectangle_to_polygon2D(Vector2D(10, 0), 5.0, 5.0);
        W.add_obstacle(polygon);
        polygon = Polygon2D::circle_to_polygon2D(2.0, 8);
        polygon = polygon.rotation(M_PI / 5.0);
        polygon = polygon.translate(Vector2D(0, 10.0));
        W.add_obstacle(polygon);
    }

    polygon = Polygon2D();
    polygon.add_vertex(Vector2D(0.0, 0));
    polygon.add_vertex(Vector2D(1.0, 0));
    polygon.add_vertex(Vector2D(0.0, 1));
    robot.set_shape(polygon);
    W.set_robot(robot);

    {
        tik = omp_get_wtime();
        W.compute_c_obstacles(SAMPLES);
        tok = omp_get_wtime();
        std::cout << "Tempo para computar o CB-obstaculo:" << tok - tik << "s\n";
    }

    std::cout << "Laço for...\n";
    double N = 1000.0;
    thr_plot = std::thread(living_plot);
    for (int i = 0; i < N; i++)
    {
        Config config;
        config.set_pos(3.0 * cos(2.0 * M_PI * i / N), 3.0 * sin(2.0 * M_PI * i / N));
        config.set_theta(2.0 * M_PI * i / N);
        W.update_config(config);
        usleep(30 * 1000);
    }
    is_finished = true;
    thr_plot.join();

    std::cout << "Programa encerrando...\n";
    return 0;
}

void living_plot()
{
    std::cout << "Living plot on!\n";

    std::list<Polygon2D> obstacles = W.get_obstacles();
    std::list<Polygon2D> cb_obstacles;
    std::vector<double> x_vec, y_vec, x_robot, y_robot;
    std::vector<Config> config_hist;
    std::vector<std::tuple<
        std::vector<double>,
        std::vector<double>>>
        pts_obs, pts;
    Gnuplot gp_work;
    Gnuplot gp_curr_config;
    auto plots_work = gp_work.plotGroup();
    auto plots_config = gp_curr_config.plotGroup();

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
            polygon_to_vectorsXY(*polygon_it, x_vec, y_vec);
            pts_obs.emplace_back(std::make_tuple(x_vec, y_vec));
        }
    }

    // Gnuplot *gp_config = new Gnuplot();
    // plot_config_space(W, gp_config, SAMPLES);
    
    while (is_finished == false)
    {
        auto plots_work = gp_work.plotGroup();
        auto plots_config = gp_curr_config.plotGroup();

        //objs
        plots_work.add_plot2d(pts_obs, "with filledcurve fc 'black'");

        // plot robot trajectory
        config_hist.push_back(W.get_robot().get_config());
        x_robot.push_back(W.get_robot().get_config().get_pos().x());
        y_robot.push_back(W.get_robot().get_config().get_pos().y());
        pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_work.add_plot2d(pts, "with line lc 'red'");

        //plot current position only
        x_vec.clear();
        y_vec.clear();
        polygon_to_vectorsXY(W.get_robot().to_polygon2D(), x_vec, y_vec);
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
            polygon_to_vectorsXY(*cb_it, x_vec, y_vec);
            pts.emplace_back(std::make_tuple(x_vec, y_vec));
        }
        plots_config.add_plot2d(pts, "with filledcurve fc 'black'");

        // robot at config space
        pts.clear();
        pts.emplace_back(std::make_tuple(x_robot, y_robot));
        plots_config.add_plot2d(pts, "with line lc 'red'");

        //show plot
        gp_work << plots_work;
        gp_curr_config << plots_config;
    }

    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
    std::cout << "Living plot off!\n";

    // if (gp_config != NULL)
    //     delete gp_config;
}

void plot_config_space(World w, Gnuplot *gp, const unsigned int n_samples)
{
    // Gnuplot gp;
    if (gp == NULL)
        return;

    std::vector<std::tuple<
        std::vector<double>,
        std::vector<double>,
        std::vector<double>>>
        pts;
    std::vector<double> x_vec, y_vec, z_vec;
    std::vector<double> x,y,z;
    Polygon2D polygon;
    auto plots = gp->splotGroup();
    // auto cb_obstacles = w.get_cobstacles(n_samples);

    *gp << "set title 'Espaço de configuração'\n";
    *gp << "load 'config_plot3D'\n";

    const double step_th = 2.0 * M_PI / n_samples;
    long i = 0;
    for (double th = 0.0; th < 2.0 * M_PI; th += step_th)
    {
        w.update_config(Config(Vector2D(), th));
        auto cb_obstacles = w.get_cobstacles(n_samples);
        for (auto cb_it = cb_obstacles.begin(); cb_it != cb_obstacles.end(); cb_it++)
        {
            x_vec.clear();
            y_vec.clear();
            polygon_to_vectorsXY(*cb_it, x_vec, y_vec);
            z_vec = std::vector<double>(x_vec.size(), th);
            pts.emplace_back(std::make_tuple(x_vec, y_vec, z_vec));
        }
        plots.add_plot2d(pts, "with polygons fc 'gray75'");
    }
    *gp << plots;
}


void polygon_to_vectorsXY(const Polygon2D &polygon, std::vector<double> &vertices_x, std::vector<double> &vertices_y)
{
    auto vertices = polygon.get_vertices();

    for (auto it = vertices.begin(); it != vertices.end(); it++)
    {
        vertices_x.push_back(it->x());
        vertices_y.push_back(it->y());
    }

    if (vertices.size() > 1.0)
    {
        vertices_x.push_back(vertices.front().x());
        vertices_y.push_back(vertices.front().y());
    }
}
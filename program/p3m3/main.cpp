#include "p3m3.hpp"
#include <string>


int main(int argc, char*argv[])
{
    Simulation_p3m3 sim;

    if(argc != 3)
    {
        std::cout << "Informe o ganho angular e linear para o controlador seguidor de caminho.\n";
        return 1;
    }

    std::cout << "Running...\n";
    sim.start_simulation("/scenes/p3m3.ttt", std::stod(argv[1]), std::stod(argv[2]));

    std::cout << "Press enter to stop the simulation." << std::endl;
    std::cin.get();

    sim.stop_simulation();
    std::cout << "Finished!\n";
    return 0;
}

#include "p3m2.hpp"


int main(int argc, char*argv[])
{
    Simulation_p3m2 sim;

    std::cout << "Running...\n";
    sim.start_simulation("scenes/p3m2.ttt", 9999999);

    std::cout << "Press enter to stop the simulation." << std::endl;
    std::cin.get();

    sim.stop_simulation();
    std::cout << "Finished!\n";
    return 0;
}

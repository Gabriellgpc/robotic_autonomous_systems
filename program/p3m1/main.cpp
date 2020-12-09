#include <iostream>
#include "p3m1.hpp"

#include <string>
#include <fstream>

using namespace std;

int main(int argc, char*argv[])
{
    Simulation_p3m1 sim;

    std::cout << "Running...\n";
    sim.start_simulation("scenes/p3m1.ttt", 9999999);

    std::cout << "Press enter to stop the simulation." << std::endl;
    std::cin.get();

    sim.stop_simulation();
    std::cout << "Finished!\n";
    return 0;
}
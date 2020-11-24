#include <iostream>
#include "p3m1.hpp"

using namespace std;

int main(int argc, char*argv[])
{
    Simulation_p3m1 sim;

    std::cout << "Running...\n";
    sim.start_simulation("scenes/p3m1.ttt", 60.0);
    std::cout << "Finished!\n";

    return 0;
}
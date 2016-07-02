#include "Util.h"
#include "Simulation.h"

int main(int argc, char *argv[]) {
    Simulation sim("results.csv");
    NewtonBody *objBody = sim.LoadObject("obj51.3ds");
    NewtonBody *toolBody = sim.LoadTool("obj52.3ds");

    sim.Start();

    std::cout << "Running Simulation ( this will take a while )\n";

    while (!sim.IsFinished()) {

        if (!sim.IterateScenario()) {
            sim.SaveResults();
            sim.NextScenario();
        }
        // update the the state of all bodies in the scene
        sim.UpdatePhysics();
    }
}


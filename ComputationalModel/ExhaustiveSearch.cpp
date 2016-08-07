#include "Util.h"
#include "ExhaustiveSimulation.h"

int main(int argc, char *argv[]) {

    //take input parameters
    const char *objModelFileName  = "obj51.3ds";
    const char *toolModelFileName = "obj52.3ds";

    if(argc==3){
        std::cout << "Loading models from input parameters\n\r";
        objModelFileName = argv[1];
        toolModelFileName = argv[2];
    }
    else std::cout << "Using default tool and object models\n\r";

    std::cout << "Object model : " << objModelFileName << "\n\r";
    std::cout << "Tool   model : " << toolModelFileName << "\n\r";


    //run exhaustive search simulation
    ExhaustiveSimulation sim("results.csv");
    NewtonBody *objBody  = sim.LoadObject(objModelFileName);
    NewtonBody *toolBody = sim.LoadTool(toolModelFileName);

    sim.Start();

    std::cout << "Running Simulation ( this will take a while )\n";

    while (!sim.IsFinished()) {
        // update the the state of all bodies in the scene
        sim.UpdatePhysics();

        //check if scenario ended
        if(!sim.IterateScenario())
        {
            sim.SaveResults();
            sim.NextScenario();
        }
    }
}


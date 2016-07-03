#include "Util.h"
#include "Simulation.h"
#include "GraphicsManager.h"

int main(int argc, char * argv[]) {
    Simulation sim("results.csv");
    NewtonBody *objBody = sim.LoadObject("obj51.3ds");
    NewtonBody *toolBody = sim.LoadTool("obj52.3ds");

    sim.Start();

    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);
    graphicsManager.Register(toolBody, GREEN);
    graphicsManager.Register(objBody, GRAY);
    graphicsManager.SetCamera(dVector(-2, 2, 2), -30, 45);
    graphicsManager.TogglePause();//pause the simulation but leave graphics rendering running

    std::cout << "Running Simulation ( this will take a while )\n";

    while (!sim.IsFinished()) {

        if (!graphicsManager.IsPhysicsPaused()) {
            // update the the state of all bodies in the scene
            sim.UpdatePhysics();

            //check if scenario ended
            if(!sim.IterateScenario())
            {
                sim.SaveResults();
                sim.NextScenario();
            }
        }

        if(!graphicsManager.IsWindowClosed())
            graphicsManager.UpdateGraphics(sim.GetTimeStep());
        else break;

    }
}


#include "Util.h"
#include "Simulation.h"
#include "PhysicsUtils.h"
#include "GraphicsManager.h"

int main() {
    Simulation sim;
    NewtonBody *objBody = sim.LoadObject("obj51.3ds");
    NewtonBody *toolBody = sim.LoadTool("obj52.3ds");

    sim.ResetTimer();
    sim.NextScenario();

    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);
    graphicsManager.Register(toolBody, GREEN);
    graphicsManager.Register(objBody, GRAY);
    graphicsManager.SetCamera(dVector(-2, 2, 2), -30, 45);
    graphicsManager.TogglePause();//pause the simulation but leave graphics rendering running

    while (!sim.IsFinished()) {
        dFloat timeStep;

        if (!graphicsManager.IsPhysicsPaused()) {

            if(!sim.IterateScenario())
            {
                sim.SaveResults();
                sim.NextScenario();
//                graphicsManager.TogglePause();
            }

            // update the the state of all bodies in the scene
            sim.UpdatePhysics();
            timeStep = dFloat(dGetTimeInMicrosenconds() - sim.GetSimulationTime());
        }

        if(!graphicsManager.IsWindowClosed()){
            graphicsManager.UpdateGraphics(timeStep);
        }
        else{
//            if(graphicsManager.IsPhysicsPaused()){
                break;
//            }
        }
    }
}


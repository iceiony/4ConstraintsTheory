#include "Util.h"
#include "Simulation.h"
#include "GraphicsManager.h"

int main(int argc, char * argv[]) {
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


    //run exhaustive simulation in graphics mode
    Simulation sim("results.csv");
    NewtonBody *objBody  = sim.LoadObject(objModelFileName);
    NewtonBody *toolBody = sim.LoadTool(toolModelFileName);

    sim.Start();

    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);
    graphicsManager.Register(toolBody, GREEN);
    graphicsManager.Register(objBody, GRAY);
    graphicsManager.SetCamera(dVector(-2, 2, 2), -30, 45);
    graphicsManager.TogglePause();//pause the simulation but leave graphics rendering running
    graphicsManager.ToggleAdditionalRender(); // render AABB box, collision frame and object XYZ center normals

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


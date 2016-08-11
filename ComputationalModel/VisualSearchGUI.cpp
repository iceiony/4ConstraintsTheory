#include <rendering/RayCastEntity.h>
#include "Util.h"
#include "GraphicsManager.h"
#include "VisualSimulation.h"

int main(int argc, char * argv[]) {
//    const char *objModelFileName  = "../ToolAndObjectModels/obj11.3ds";
//    const char *toolModelFileName = "../ToolAndObjectModels/obj12.3ds";
     const char *objModelFileName  = "obj51.3ds";
     const char *toolModelFileName = "obj52.3ds";

    if(argc==3){
        std::cout << "Loading models from input parameters\r\n";
        objModelFileName = argv[1];
        toolModelFileName = argv[2];
    }
    else std::cout << "Using default tool and object models\r\n";

    std::cout << "Object model : " << objModelFileName << "\r\n";
    std::cout << "Tool   model : " << toolModelFileName << "\r\n";


    //run exhaustive simulation in graphics mode
    VisualSimulation sim("results.csv");
    NewtonBody *objBody  = sim.LoadObject(objModelFileName);
    NewtonBody *toolBody = sim.LoadTool(toolModelFileName);

    sim.PrepareNextScenario();

    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);
    graphicsManager.Register(toolBody, GREEN);
    graphicsManager.Register(objBody, GRAY);
    graphicsManager.SetCamera(dVector(-3, 3, 3), -30, 45);
    graphicsManager.TogglePause();//pause the simulation but leave graphics rendering running


    RayCastEntity *objView  = new RayCastEntity(sim.m_objRayStart,sim.m_objRayEnd);
    RayCastEntity *toolView = new RayCastEntity(sim.m_toolRayStart,sim.m_toolRayEnd);

    toolView->SetSubSurface(&sim.m_toolSubView);
    objView->SetSubSurface(&sim.m_objSubView);

    graphicsManager.Append(objView);
    graphicsManager.Append(toolView);

    std::cout << "Running Simulation ( this version should be faster )\n";

    bool correlationFound = false;
    while (!sim.IsFinished()) {

        if (!graphicsManager.IsPhysicsPaused()) {
            if(!correlationFound){
//                graphicsManager.TogglePause();
                sim.NextSubSurface();
                correlationFound = sim.CorrelateSubSurfaces();
                if(correlationFound){
                    sim.TryConfiguration();
                    graphicsManager.TogglePause();
                }
                else if(!sim.HasNextSubSurface()) {
                    sim.PrepareNextScenario();
//                    graphicsManager.TogglePause();
                }
            }
            else{
                sim.UpdatePhysics();
                if(!sim.IterateScenario())
                {
                    sim.ResetStateAndPosition();
                    sim.SaveResults();
                    correlationFound = false;
                }
            }
        }

        if(!graphicsManager.IsWindowClosed())
        {
            graphicsManager.UpdateGraphics(sim.GetTimeStep());
        }
        else break;
    }
}


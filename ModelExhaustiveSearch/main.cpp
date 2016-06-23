#include "Util.h"
#include "GraphicsManager.h"
#include "PhysicsUtils.h"
#include "Simulation.h"

int main() {
    Simulation sim;
    GraphicsManager graphicsManager(sim.GetNewtonWorld());

    NewtonBody* objBody = sim.LoadObject("obj51.3ds");
    NewtonBody* toolBody = sim.LoadTool("obj52.3ds");

    graphicsManager.Register(toolBody);
    graphicsManager.Register(objBody);
    graphicsManager.SetCamera(dVector(-5, 2, 0), 0, 0);


    dMatrix origin(dGetIdentityMatrix());
    origin.m_posit = dVector(0,1,0);
    NewtonBodySetMatrix(objBody,&origin[0][0]);

    origin.m_posit = dVector(-3,1.15f,0.326f);
    NewtonBodySetMatrix(toolBody,&origin[0][0]);

    //rotate tool body in proper position
    dMatrix currentPos ;
    dMatrix rotated (dPitchMatrix(180.0f * 3.1416f / 180.0f)) ;
    NewtonBodyGetMatrix(toolBody,&currentPos[0][0]);
    rotated.m_posit = currentPos.m_posit;
    NewtonBodySetMatrix(toolBody,&rotated[0][0]);

    NewtonBodySetForceAndTorqueCallback(toolBody,MoveTool);

    dVector velocity(1,0,0);
    NewtonBodySetVelocity(toolBody,&velocity[0]);

    sim.ResetTimer();

    while(!graphicsManager.IsWindowClosed()){
        // update the the state of all bodies in the scene
        sim.UpdatePhysics();

        graphicsManager.UpdateGraphics(sim.GetSimulationTime());
    }
}


#include "Util.h"
#include "Simulation.h"
#include "PhysicsUtils.h"
#include "GraphicsManager.h"

int main() {
    Simulation sim;
    NewtonBody *objBody = sim.LoadObject("obj51.3ds");
    NewtonBody *toolBody = sim.LoadTool("obj52.3ds");

    dMatrix origin(dGetIdentityMatrix());
    origin.m_posit = dVector(0, 1, 0);
    NewtonBodySetMatrix(objBody, &origin[0][0]);

    origin.m_posit = dVector(0, 2.15f, 0.326f);
    NewtonBodySetMatrix(toolBody, &origin[0][0]);

    //rotate tool body in proper position
    dMatrix currentPos;
    dMatrix rotated(dPitchMatrix(180.0f * 3.1416f / 180.0f));
    NewtonBodyGetMatrix(toolBody, &currentPos[0][0]);
    rotated.m_posit = currentPos.m_posit;
    NewtonBodySetMatrix(toolBody, &rotated[0][0]);

    NewtonBodySetForceAndTorqueCallback(toolBody, MoveTool);


    GraphicsManager graphicsManager(sim.GetNewtonWorld());

    graphicsManager.Register(sim.GetFloor(), BLUE);
    graphicsManager.Register(toolBody, GREEN);
    graphicsManager.Register(objBody, GREEN);
    graphicsManager.SetCamera(dVector(-5, 2, 0), 0, 0);

    sim.ResetTimer();

    while (!graphicsManager.IsWindowClosed()) {
        dFloat timeStep;

        //by default graphics is initially paused
        if (!graphicsManager.IsPhysicsPaused()) {
            // update the the state of all bodies in the scene
            sim.UpdatePhysics();
            timeStep = dFloat(dGetTimeInMicrosenconds() - sim.GetSimulationTime());
        }

        graphicsManager.UpdateGraphics(timeStep);
    }
}


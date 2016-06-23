#include <iostream>
#include "Util.h"
#include "DemoEntityManager.h"
#include "PhysicsUtils.h"

#define MASS  10.0f

class Simulation {

private:
    NewtonWorld *m_world;
    NewtonBody *toolBody;
    NewtonBody *objBody;

    unsigned64 m_microseconds;

    NewtonBody *CreateFloor();
    NewtonBody *LoadModel(const char *fileName);

public:
    Simulation() : m_microseconds(0),m_world(NewtonCreate()) {

        //set exact solving
        NewtonSetSolverModel(m_world, 0);

        // clean up all caches the engine have saved
        NewtonInvalidateCache(m_world);

        dTimeTrackerSetThreadName ("mainThread");

        CreateFloor();
    }

    ~Simulation() {
        // is we are run asynchronous we need make sure no update in on flight.
        if (m_world) {
            NewtonWaitForUpdateToFinish(m_world);
        }

        // destroy the empty world
        if (m_world) {
            NewtonDestroy(m_world);
            m_world = NULL;
        }
        dAssert (NewtonGetMemoryUsed() == 0);
    }

    void ResetTimer() {
        dResetTimer();
        m_microseconds = dGetTimeInMicrosenconds();
    }

    void UpdatePhysics() {
        // update the physics
        if (m_world) {

            dFloat timeStepInSeconds = 1.0f / MAX_PHYSICS_FPS;
            unsigned64 timeStepMicroseconds = unsigned64(timeStepInSeconds * 1000000.0f);

            unsigned64 currentTime = dGetTimeInMicrosenconds();
            unsigned64 nextTime = currentTime - m_microseconds;
            if (nextTime > timeStepMicroseconds * 2) {
                m_microseconds = currentTime - timeStepMicroseconds * 2;
                nextTime = currentTime - m_microseconds;
            }

            if (nextTime >= timeStepMicroseconds) {
                dTimeTrackerEvent(__FUNCTION__);
                // run the newton update function
                NewtonUpdateAsync(m_world, timeStepInSeconds);
                m_microseconds += timeStepMicroseconds;
            }
        }
    }

    unsigned64 GetSimulationTime(){
        return m_microseconds;
    }


    NewtonWorld *GetNewtonWorld();

    NewtonBody * LoadObject(const char *fileName);
    NewtonBody * LoadTool(const char *fileName);

};

NewtonBody* Simulation::CreateFloor ()
{
    dMatrix origin(dGetIdentityMatrix());
    dFloat mass = 0.0f;

    NewtonMesh * newtonMesh = CreateFloorMesh(m_world);

    // now we can use this mesh for lot of stuff, we can apply UV, we can decompose into convex,
    NewtonCollision* const collision = NewtonCreateConvexHullFromMesh(m_world, newtonMesh, 0.001f, 0);

    NewtonBody* const floorBody = CreateSimpleBody(m_world, NULL, mass, origin, collision, 0);

    NewtonDestroyCollision(collision);
    NewtonMeshDestroy (newtonMesh);

    return floorBody;
}

NewtonWorld *Simulation::GetNewtonWorld() {
    return m_world;
}

NewtonBody * Simulation::LoadModel(const char * fileName){
    NewtonMesh* const mesh = LoadMeshFrom3DS(m_world, fileName, 0.008);
    NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.00001f, 0.0f, 256, 100, nullptr, nullptr);
    NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (m_world, convexApproximation, 0.001f, 0, 0);

    dMatrix position (dGetIdentityMatrix());

    NewtonBody *modelBody = CreateSimpleBody(m_world, NULL , MASS , position, compound, 0);

    NewtonDestroyCollision(compound);
    NewtonMeshDestroy(convexApproximation);
    NewtonMeshDestroy(mesh);

    return modelBody;
}

NewtonBody * Simulation::LoadObject(const char *fileName) {
    toolBody = LoadModel(fileName);
    return toolBody;
}

NewtonBody * Simulation::LoadTool(const char *fileName) {
    objBody = LoadModel(fileName);
    return objBody;
}

int main() {
    Simulation sim;
    DemoEntityManager graphicsManager(sim.GetNewtonWorld());

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


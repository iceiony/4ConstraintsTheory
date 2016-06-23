//
// Created by Adrian Ionita on 23/06/2016.
//
#include "Simulation.h"
#include "rendering/PhysicsUtils.h"

void Simulation::UpdatePhysics() {
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
    NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.00001f, 0.0f, 512, 256, nullptr, nullptr);
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
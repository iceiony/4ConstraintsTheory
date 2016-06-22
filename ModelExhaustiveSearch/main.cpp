#include <iostream>
#include "Util.h"
#include "DemoEntityManager.h"


void SimpleConvexApproximation(DemoEntityManager *const scene);

using namespace std;

class Simulation {

private:
    NewtonWorld *m_world;
    unsigned64 m_microseconds;

public:
    Simulation() : m_microseconds(0),m_world(NewtonCreate()) {

        //set exact solving
        NewtonSetSolverModel(m_world, 0);

        // clean up all caches the engine have saved
        NewtonInvalidateCache(m_world);

        dTimeTrackerSetThreadName ("mainThread");;
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
};

NewtonWorld *Simulation::GetNewtonWorld() {
    return m_world;
}

int main() {

    Simulation sim;
    DemoEntityManager graphicsManager(sim.GetNewtonWorld());

    sim.ResetTimer();

    SimpleConvexApproximation(&graphicsManager);

    while(!graphicsManager.IsWindowClosed()){
        // update the the state of all bodies in the scene
        sim.UpdatePhysics();

        graphicsManager.UpdateGraphics(sim.GetSimulationTime());
    }
}


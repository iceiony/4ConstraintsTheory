//
// Created by Adrian Ionita on 23/06/2016.
//
#include <iostream>
#include "Util.h"
#include "dHighResolutionTimer.h"

#define MASS  10.0f
#ifndef __SIMULATION__
#define __SIMULATION__
class Simulation {

private:
    NewtonWorld *m_world;
    NewtonBody *toolBody;
    NewtonBody *objBody;
    NewtonBody *floorBody;

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

    void UpdatePhysics();


    unsigned64 GetSimulationTime(){
        return m_microseconds;
    }


    NewtonWorld *GetNewtonWorld();

    NewtonBody * LoadObject(const char *fileName);
    NewtonBody * LoadTool(const char *fileName);

    NewtonBody *GetFloor();

};
#endif

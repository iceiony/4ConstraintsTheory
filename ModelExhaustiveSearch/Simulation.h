//
// Created by Adrian Ionita on 23/06/2016.
//
#include <iostream>
#include "Util.h"
#include <iostream>
#include <fstream>

#define MASS  10.0f
#ifndef __SIMULATION__
#define __SIMULATION__

#define maxIterationCount 12

#define minX -1
//#define minX 0.6
#define maxX 1

#define minY 1.16
#define maxY 2

#define minZ 0.3
//#define minZ -.5f
#define maxZ 1

#define minYaw 0
#define maxYaw 360

#define minPitch 180
//#define minPitch 90
#define maxPitch 360

#define minRoll 0
#define maxRoll 360

class Simulation {

private:
    NewtonWorld *m_world;
    NewtonBody *m_toolBody;
    NewtonBody *m_objBody;
    NewtonBody *m_floorBody;

    dVector m_objInitialPos;

    int iterationCount;
    bool isPossibleSolution;
    std::ofstream m_output;

    NewtonBody *CreateFloor();
    NewtonBody *LoadModel(const char *fileName);
    float GetMinY(NewtonBody *body);

public:
    Simulation(const char * const outputFile);

    ~Simulation();

    void UpdatePhysics();

    NewtonWorld *GetNewtonWorld();

    NewtonBody * LoadObject(const char *fileName);
    NewtonBody * LoadTool(const char *fileName);

    NewtonBody *GetFloor();

    bool IsFinished();

    void NextScenario();

    void ResetObjPosition();

    void SetToolRotation(float yaw, float pitch, float roll,float x, float y, float z);

    bool IterateScenario();

    void SaveResults();
    float offsetX = minX;
    float offsetY = minY;
    float offsetZ = minZ;

    int offsetYaw   = minYaw;
    int offsetPitch = minPitch;
    int offsetRoll  = minRoll;

    float GetTimeStep();
};
#endif

//
// Created by Adrian Ionita on 23/06/2016.
//
#include <iostream>
#include "Util.h"
#include <iostream>
#include <fstream>

#ifndef __SIMULATION__
#define __SIMULATION__

#define MASS  10.0f

#define maxIterationCount 1

#define minYaw 0
#define maxYaw 360

#define minPitch 0
#define maxPitch 360

#define minRoll 0
#define maxRoll 360

#define rotationStep 5
#define positionStep 0.02f
#define marginFactor 3.0f

class Simulation {

private:
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;

    NewtonWorld *m_world;
    NewtonBody *m_toolBody;
    NewtonBody *m_objBody;
    NewtonBody *m_floorBody;

    dVector m_objInitialPos;

    int iterationCount;
    bool isPossibleSolution;
    std::ofstream m_output;

    NewtonBody *CreateFloor();
    void ReadjustMinMaxLimits();

public:
    Simulation(const char * const outputFile);

    ~Simulation();

    void UpdatePhysics();

    NewtonWorld *GetNewtonWorld();

    NewtonBody *LoadModel(const char *fileName);
    NewtonBody * LoadObject(const char *fileName);
    NewtonBody * LoadTool(const char *fileName);

    NewtonBody *GetFloor();

    bool IsFinished();

    void NextScenario();

    void ResetObjPosition();

    void SetToolParameters(float yaw, float pitch, float roll, float x, float y, float z);

    bool IterateScenario();

    void SaveResults();

    float offsetX = .0f;
    float offsetY = .0f;
    float offsetZ = .0f;

    int offsetYaw   = minYaw;
    int offsetPitch = minPitch;
    int offsetRoll  = minRoll;

    float GetTimeStep();


    void Start();
};
#endif

//
// Created by Adrian Ionita on 23/06/2016.
//
#include <iostream>
#include "Util.h"
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

#ifndef __VISUAL_SIMULATION__
#define __VISUAL_SIMULATION__

#define MASS  10.0f

#define maxIterationCount 1

#define rotationStep 45
#define positionStep 0.02f
#define marginFactor 3.0f

class VisualSimulation {

private:
    NewtonWorld *m_world;
    NewtonBody *m_toolBody;
    NewtonBody *m_objBody;
    NewtonBody *m_floorBody;

    dVector m_objInitialPos = dVector(0.0f);
    dVector m_toolInitialPos = dVector(.0f,.0f,2.0f) ;

    dFloat m_toolYaw = -rotationStep;
    dFloat m_toolPitch;
    dFloat m_objYaw;

    int iterationCount;
    bool isPossibleSolution;
    std::ofstream m_output;

    NewtonBody *CreateFloor();

public:
    VisualSimulation(const char * const outputFile);

    ~VisualSimulation();

    void UpdatePhysics();

    NewtonWorld *GetNewtonWorld();

    NewtonBody * LoadModel(const char *fileName);
    NewtonBody * LoadObject(const char *fileName);
    NewtonBody * LoadTool(const char *fileName);

    NewtonBody *GetFloor();

    bool IsFinished();

    bool IterateScenario();

    void SaveResults();

    float GetTimeStep();

    void PrepareNextScenario();

    void ResetObjectPositionMatrix();
    void ResetToolPositionMatrix();

    vector<dVector> * m_toolRayStart = new vector<dVector>();
    vector<dVector> * m_toolRayEnd = new vector<dVector>();
    int m_toolRowCount;
    int m_toolColCount;

    vector<dVector> * m_objRayStart=new vector<dVector>();
    vector<dVector> * m_objRayEnd=new vector<dVector>();
    int m_objRowCount;
    int m_objColCount;

    std::tuple<int, int> GetRaycastSurfaces(dVector minP, dVector maxP, vector<dVector> *start, vector<dVector> *end) ;

    //sub surfaces for the whole object view
    unsigned int m_toolMinX;
    unsigned int m_toolMinY;
    unsigned int m_objMinX;
    unsigned int m_objMinY;

    array<int,VIEW_DIMENSION * VIEW_DIMENSION> m_toolSubView;
    array<int,VIEW_DIMENSION * VIEW_DIMENSION> m_objSubView;

    int m_subSurfaceAttempt = 0;

    void NextSubSurface();
    bool HasNextSubSurface();

    void ResetStateAndPosition();

    bool CorrelateSubSurfaces();

    void TryConfiguration();
};
#endif

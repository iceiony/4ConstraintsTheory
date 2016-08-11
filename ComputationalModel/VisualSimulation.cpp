//
// Created by Adrian Ionita on 23/06/2016.
//
#include <vector>
#include <array>
#include <tuple>
#include "VisualSimulation.h"
#include "PhysicsUtils.h"
#include "statistics.h"

using namespace alglib;
using namespace std;

VisualSimulation::VisualSimulation(const char *const outputFile) : m_world(NewtonCreate()) {

    NewtonSetThreadsCount(m_world, NewtonGetMaxThreadsCount(m_world));

    printf("NewtonDynamicsVersion : %d\n", NewtonWorldGetVersion());
    printf("NewtonThreadCount : %d \n", NewtonGetThreadsCount(m_world));

    //set exact solving
    NewtonSetSolverModel(m_world, 0);

    // clean up all caches the engine have saved
    NewtonInvalidateCache(m_world);

    dTimeTrackerSetThreadName ("mainThread");

    CreateFloor();

    if(outputFile){
        m_output.open(outputFile);
        m_output << "x,y,z,yaw,pitch,roll\n";
    }
}

VisualSimulation::~VisualSimulation() {

    if (m_world) {
        NewtonWaitForUpdateToFinish(m_world);
        NewtonDestroy(m_world);
        m_world = NULL;
    }
    dAssert (NewtonGetMemoryUsed() == 0);
}

void VisualSimulation::UpdatePhysics() {
    // update the physics
    if (m_world) {
        try {
            NewtonUpdate(m_world, this->GetTimeStep());
        }
        catch (...){
            std::cout<<"Segmentation Fault ";
            PrintTime();
        }
    }
}

NewtonBody *VisualSimulation::CreateFloor() {
    dMatrix origin(dGetIdentityMatrix());
    dFloat mass = 0.0f;

    NewtonMesh *newtonMesh = CreateFloorMesh(m_world);

    // now we can use this mesh for lot of stuff, we can apply UV, we can decompose into convex,
    NewtonCollision *const collision = NewtonCreateConvexHullFromMesh(m_world, newtonMesh, 0.001f, 0);

    m_floorBody = CreateSimpleBody(m_world, NULL, mass, origin, collision, 0);

    NewtonDestroyCollision(collision);
    NewtonMeshDestroy(newtonMesh);

    return m_floorBody;
}

NewtonWorld *VisualSimulation::GetNewtonWorld() {
    return m_world;
}

NewtonBody *VisualSimulation::LoadModel(const char *fileName) {
    NewtonMesh *const mesh = LoadMeshFrom3DS(m_world, fileName, 0.008);
    NewtonMesh *const convexApproximation = NewtonMeshApproximateConvexDecomposition(mesh, 0.00001f, 0.0f, 512, 256,
                                                                                     nullptr, nullptr);
    NewtonCollision *const compound = NewtonCreateCompoundCollisionFromMesh(m_world, convexApproximation, 0.001f, 0, 0);

    NewtonBody *modelBody = CreateSimpleBody(m_world, NULL, MASS, dGetZeroMatrix() , compound, 0);

    NewtonDestroyCollision(compound);
    NewtonMeshDestroy(convexApproximation);
    NewtonMeshDestroy(mesh);

    return modelBody;
}

NewtonBody *VisualSimulation::LoadTool(const char *fileName) {
    m_toolBody = this->LoadModel(fileName);

    // set the force and torque call back function
    NewtonBodySetForceAndTorqueCallback(m_toolBody, MoveTool);

    ResetToolPositionMatrix();

    return m_toolBody;
}

NewtonBody *VisualSimulation::LoadObject(const char *fileName) {
    m_objBody = this->LoadModel(fileName);

    // set the force and torque call back function
    NewtonBodySetForceAndTorqueCallback(m_objBody, PhysicsApplyGravityForce);

    ResetObjectPositionMatrix();

    return m_objBody;
}

void VisualSimulation::PrepareNextScenario() {
    //compute next rotations
    if(m_toolYaw < 360)
        m_toolYaw += rotationStep;
    else
        m_toolPitch += rotationStep;

    if(m_toolPitch >= 360 && m_objYaw<360) {
        m_objYaw += rotationStep;
        m_toolPitch = 0;
        m_toolYaw = 0;
    }

    //update to new rotations
    ResetStateAndPosition();

    //get raycast surface for entire AABB block
    dVector minP,maxP;
    CalculateAABB(m_toolBody, minP, maxP);
    std::tie(m_toolRowCount,m_toolColCount) = GetRaycastSurfaces(minP,maxP,m_toolRayStart,m_toolRayEnd);

    CalculateAABB(m_objBody, minP, maxP);
    std::tie(m_objRowCount,m_objColCount) = GetRaycastSurfaces(minP,maxP,m_objRayStart,m_objRayEnd);

    m_subSurfaceAttempt = 0;
    m_toolSubView.fill(-1);
    m_objSubView.fill(-1);

    UpdatePhysics();
    UpdatePhysics();
}


NewtonBody *VisualSimulation::GetFloor() {
    return m_floorBody;
}

bool VisualSimulation::IsFinished() {
//    return offsetRoll > maxRoll;
}

bool VisualSimulation::IterateScenario() {


    //check if objects do not penetrate each other
    if(!IsSmallPenetration(m_toolBody,m_objBody,0.02f)){
        return false;
    }

    //if object enters sleep state then scenario failed
    if (NewtonBodyGetSleepState(m_objBody) == 1){
        return false;
    }

    //if objects no velocity transferred whilst lifting then scenario failed
    dVector objVelocity;
    NewtonBodyGetVelocity(m_objBody, &objVelocity[0]);

    if(objVelocity.m_y < 0.5f){
        return false;
    }

    isPossibleSolution = true;

    //allow a few more iterations for presentation
    return iterationCount < maxIterationCount;
}


void VisualSimulation::SaveResults() {
//    if (isPossibleSolution) {
//        std::cout << "x:" << offsetX << " y:" << offsetY << " z:" << offsetZ;
//        std::cout << " yaw:" << offsetYaw << " pitch:" << offsetPitch << " roll:" << offsetRoll << '\n';
//        std::cout.flush();
//
//        m_output << offsetX << ',' << offsetY << ',' << offsetZ << ',';
//        m_output << offsetYaw << ',' << offsetPitch << ',' << offsetRoll << '\n';
//
//        m_output.flush();
//    }
}

float VisualSimulation::GetTimeStep() {
    return 1.0f / MAX_PHYSICS_FPS;
}

void VisualSimulation::ResetObjectPositionMatrix() {
    dMatrix origin(dYawMatrix(radians(m_objYaw)));
    origin.m_posit = m_objInitialPos;
    NewtonBodySetMatrix(m_objBody, &origin[0][0]);

    //position lowest point on the floor
    dVector minP, maxP;
    CalculateAABB(m_objBody, minP, maxP);

    if(minP.m_y<0){
        NewtonBodyGetMatrix(m_objBody,&origin[0][0]);
        origin.m_posit.m_y += .0f - minP.m_y;
        NewtonBodySetMatrix(m_objBody, &origin[0][0]);
    }

}

void VisualSimulation::ResetToolPositionMatrix() {
    dMatrix origin(dPitchMatrix(radians(m_toolPitch)) * dYawMatrix(radians(m_toolYaw)));
    origin.m_posit = m_toolInitialPos;
    NewtonBodySetMatrix(m_toolBody, &origin[0][0]);

    //position lowest point on the floor
    dVector minP, maxP;
    CalculateAABB(m_toolBody, minP, maxP);

    if(minP.m_y<0){
        NewtonBodyGetMatrix(m_toolBody,&origin[0][0]);
        origin.m_posit.m_y += .0f - minP.m_y;
        NewtonBodySetMatrix(m_toolBody, &origin[0][0]);
    }

}

std::tuple<int, int> VisualSimulation::GetRaycastSurfaces(dVector minP, dVector maxP, vector<dVector> *start, vector<dVector> *end) {
    unsigned int rowCount = ceil((maxP.m_y - minP.m_y) / CAST_STEP);
    unsigned int colCount = ceil((maxP.m_z - minP.m_z) / CAST_STEP);

    start->clear();
    end->clear();

    for (float r = minP.m_y; r < maxP.m_y ; r += CAST_STEP) {
        for(float c = minP.m_z; c < maxP.m_z ; c += CAST_STEP){
            //initialise ray at a slight angle downwards
            dVector startPoint = dVector(-1.5f,r + 0.2f,c);
            dVector endPoint = dVector(+1.5f,r - 0.2f,c);

            //calculate intersection points
            dFloat scaleParam(1.1f);
            NewtonWorldRayCast(m_world, &startPoint[0], &endPoint[0], RayCast, &scaleParam, NULL, 1);
            endPoint = startPoint + (endPoint - startPoint).Scale(scaleParam);

            //add to raycast vector
            start->push_back(startPoint);
            end->push_back(endPoint);
        }
    }

    std::tuple<int ,int> out(rowCount,colCount);
    return out;
}

/**
 * Select random sub surfaces for the current rotation to correlate on
 */
void VisualSimulation::NextSubSurface() {
    m_subSurfaceAttempt++;

    m_toolMinX += rand() % ( m_toolColCount - VIEW_DIMENSION);
    m_toolMinY += rand() % ( m_toolRowCount - VIEW_DIMENSION);
    m_toolMinX = m_toolMinX + VIEW_DIMENSION > m_toolColCount ? 0 : m_toolMinX;
    m_toolMinY = m_toolMinY + VIEW_DIMENSION > m_toolRowCount ? 0 : m_toolMinY;

    m_objMinX += rand() % ( m_objColCount - VIEW_DIMENSION);
    m_objMinY += rand() % ( m_objRowCount - VIEW_DIMENSION);
    m_objMinX = m_objMinX + VIEW_DIMENSION > m_objColCount ? 0 : m_objMinX;
    m_objMinY = m_objMinY + VIEW_DIMENSION > m_objRowCount ? 0 : m_objMinY;

    //select indices of the points defining the sub-surfaces
    m_toolSubView.fill(0);
    m_objSubView.fill(0);
    for(int i=0;i<VIEW_DIMENSION;i++){
        for(int j=0;j<VIEW_DIMENSION;j++){
            m_toolSubView[i*VIEW_DIMENSION+j] = (m_toolMinY + i) * m_toolColCount + m_toolMinX + j;
            m_objSubView[i*VIEW_DIMENSION+j]  = (m_objMinY + i) * m_objColCount + m_objMinX + j;
        }
    }
}


bool VisualSimulation::HasNextSubSurface() {
    return m_subSurfaceAttempt <= 20;
}

void VisualSimulation::ResetStateAndPosition() {
    ResetToolPositionMatrix();
    ResetObjectPositionMatrix();
    //remove newton_world state after moving the objects
    NewtonWaitForUpdateToFinish(m_world);
    NewtonInvalidateCache(m_world);
    //remove any lingering forces
    NewtonBodySetVelocity(m_toolBody, &dVector(.0f)[0]);
    NewtonBodySetOmega(m_toolBody, &dVector(.0f)[0]);

    NewtonBodySetVelocity(m_objBody, &dVector(.0f)[0]);
    NewtonBodySetOmega(m_objBody, &dVector(0.0f)[0]);
}

bool VisualSimulation::CorrelateSubSurfaces() {
//    std::ofstream objFile;
//    std::ofstream toolFile;
//    objFile.open("./surfaces/obj.csv");
//    toolFile.open("./surfaces/tool.csv");

    real_1d_array objX,objY,objZ;
    real_1d_array toolX,toolY,toolZ;
    
    unsigned total = VIEW_DIMENSION*VIEW_DIMENSION;
    
    objX.setlength(total);
    objY.setlength(total);
    objZ.setlength(total);

    toolX.setlength(total);
    toolY.setlength(total);
    toolZ.setlength(total);

    for(int i=0;i<total;i++){
        dVector objPoint = m_objRayEnd->at(m_objSubView[i]);
        dVector toolPoint = m_toolRayEnd->at(m_toolSubView[i]);

//       objFile << objPoint.m_x << ',' << objPoint.m_y << ',' << objPoint.m_z << "\n";
//       toolFile << toolPoint.m_x << ',' << toolPoint.m_y << ',' << toolPoint.m_z << "\n";

        objX[i] = objPoint.m_x;
        objY[i] = objPoint.m_y;
        objZ[i] = objPoint.m_z;

        toolX[i] = toolPoint.m_x;
        toolY[i] = toolPoint.m_y;
        toolZ[i] = toolPoint.m_z;

    }

    double xCorr = pearsoncorr2(objX,toolX);
    double yCorr = pearsoncorr2(objY,toolY);
    double zCorr = pearsoncorr2(objZ,toolZ);
    double fullCorr = xCorr * yCorr * zCorr;

//    objFile.flush();
//    objFile.close();
//    toolFile.flush();
//    toolFile.close();

    if(fullCorr < -0.2f){
        cout << fullCorr << "\r\n";
    }

    return fullCorr < -0.2f;
}

void VisualSimulation::TryConfiguration() {

}



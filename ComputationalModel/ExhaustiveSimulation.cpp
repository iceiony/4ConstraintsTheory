//
// Created by Adrian Ionita on 23/06/2016.
//
#include "Util.h"
#include "ExhaustiveSimulation.h"
#include "PhysicsUtils.h"
#include <iostream>
#ifdef _MSC_VER
#include <windows.h>
#endif

using namespace std;

ExhaustiveSimulation::ExhaustiveSimulation(const char *const outputFile) : m_world(NewtonCreate()) {

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

ExhaustiveSimulation::~ExhaustiveSimulation() {
    if (m_world) {
        NewtonWaitForUpdateToFinish(m_world);
        NewtonDestroy(m_world);
        m_world = NULL;
    }
    dAssert (NewtonGetMemoryUsed() == 0);
}

void ExhaustiveSimulation::UpdatePhysics() {
    // update the physics
    if (m_world) {
        try {
            NewtonUpdate(m_world, this->GetTimeStep());
        }
        catch (...){
            cout<<"Segmentation Fault ";
            PrintTime();
        }
    }
}

NewtonBody *ExhaustiveSimulation::CreateFloor() {
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

NewtonWorld *ExhaustiveSimulation::GetNewtonWorld() {
    return m_world;
}

NewtonBody *ExhaustiveSimulation::LoadModel(const char *fileName) {
    NewtonMesh *const mesh = LoadMeshFrom3DS(m_world, fileName, 0.008);
    NewtonMesh *const convexApproximation = NewtonMeshApproximateConvexDecomposition(mesh, 0.00001f, 0.0f, 512, 256,
                                                                                     nullptr, nullptr);
    NewtonCollision *const compound = NewtonCreateCompoundCollisionFromMesh(m_world, convexApproximation, 0.001f, 0, 0);

    NewtonBody *modelBody = CreateSimpleBody(m_world, NULL, MASS, dGetIdentityMatrix(), compound, 0);

    NewtonDestroyCollision(compound);
    NewtonMeshDestroy(convexApproximation);
    NewtonMeshDestroy(mesh);

    return modelBody;
}

NewtonBody *ExhaustiveSimulation::LoadTool(const char *fileName) {
    m_toolBody = this->LoadModel(fileName);

    // set the force and torque call back function
    NewtonBodySetForceAndTorqueCallback(m_toolBody, MoveTool);

    return m_toolBody;
}

NewtonBody *ExhaustiveSimulation::LoadObject(const char *fileName) {
    m_objBody = this->LoadModel(fileName);

    // set the force and torque call back function
    NewtonBodySetForceAndTorqueCallback(m_objBody, PhysicsApplyGravityForce);

    //position lowest point on the floor
    dVector minP, maxP;
    CalculateAABB(m_objBody, minP, maxP);

    m_objInitialPos = dVector(.0f);
    m_objInitialPos.m_y = .0f - minP.m_y;

    return m_objBody;
}

void ExhaustiveSimulation::Start() {
    ResetObjPosition();
    SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);
    ReadjustMinMaxLimits();
    SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);
}

void ExhaustiveSimulation::ReadjustMinMaxLimits() {//set min/max values for simulation search based on current rotation
    //calculate new min/max for xyz coordinates
    dVector toolMinP, toolMaxP, objMinP, objMaxP;
    CalculateAABB(m_toolBody, toolMinP, toolMaxP);
    CalculateAABB(m_objBody, objMinP, objMaxP);

    dMatrix location;
    NewtonBodyGetMatrix(m_objBody, &location[0][0]);
    dVector objPosition = location.m_posit;

    NewtonBodyGetMatrix(m_toolBody, &location[0][0]);
    dVector toolPosition = location.m_posit;

    minX = (objMinP.m_x - objPosition.m_x) + (toolPosition.m_x - toolMaxP.m_x);
    maxX = (objMaxP.m_x - objPosition.m_x) + (toolPosition.m_x - toolMinP.m_x);

    minY = (objMinP.m_y - objPosition.m_y) + (toolPosition.m_y - toolMaxP.m_y);
    //set object above floor
    minY = max(minY, .0f - toolMinP.m_y);
    maxY = (objMaxP.m_y - objPosition.m_y) + (toolPosition.m_y - toolMinP.m_y);

    minZ = (objMinP.m_z - objPosition.m_z) + (toolPosition.m_z - toolMaxP.m_z);
    maxZ = (objMaxP.m_z - objPosition.m_z) + (toolPosition.m_z - toolMinP.m_z);

    minX += positionStep * marginFactor;
    maxX -= positionStep * marginFactor;

    maxY -= positionStep * marginFactor;

    minZ += positionStep * marginFactor;
    maxZ -= positionStep * marginFactor;

//    reset offsets
    offsetX = minX ;
    offsetY = minY ;
    offsetZ = minZ ;
//    offsetX = minX + .6f;
//    offsetY = 1.14f;
//    offsetZ = 0.32f;
}

NewtonBody *ExhaustiveSimulation::GetFloor() {
    return m_floorBody;
}

bool ExhaustiveSimulation::IsFinished() {
    return offsetRoll > maxRoll;
}

void ExhaustiveSimulation::ResetObjPosition() {
    dMatrix origin(dGetIdentityMatrix());
    origin.m_posit = m_objInitialPos;
    NewtonBodySetMatrix(m_objBody, &origin[0][0]);
}

void ExhaustiveSimulation::NextScenario() {
    this->ResetObjPosition();

    //increment position values
    offsetX += positionStep;

    if (offsetX > maxX) {
        offsetX = minX;
        offsetY += positionStep;
    }

    if (offsetY > maxY) {
        offsetY = minY;
        offsetZ += positionStep;
    }

    //increment rotation values
    bool isRotationChanged = false;

    if (offsetZ > maxZ) {
        offsetZ = minZ;
        offsetYaw += rotationStep;
        isRotationChanged = true;

        cout << "Yaw increased to " << offsetYaw << '\n';
        cout.flush();
    }

    if (offsetYaw > maxYaw) {
        offsetYaw = minYaw;
        offsetPitch += rotationStep;
        isRotationChanged = true;

        cout << "Pitch increased to " << offsetPitch << '\n';
        cout.flush();
    }

    if (offsetPitch > maxPitch) {
        offsetPitch = minPitch;
        offsetRoll += rotationStep;
        isRotationChanged = true;

        cout << 100 * offsetRoll / maxRoll << "%\n";
        cout.flush();
    }

    //set tool position to new coordinates
    this->SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);

    if (isRotationChanged) {
        ReadjustMinMaxLimits();
        this->SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);
    }

    //remove newton_world state after moving the objects
    NewtonWaitForUpdateToFinish(m_world);
    NewtonInvalidateCache(m_world);

    NewtonBodySetVelocity(m_toolBody, &dVector(.0f)[0]);
    NewtonBodySetOmega(m_toolBody, &dVector(.0f)[0]);

    NewtonBodySetVelocity(m_objBody, &dVector(.0f)[0]);
    NewtonBodySetOmega(m_objBody, &dVector(0.0f)[0]);

    //reset scenario tracking
    iterationCount = 0;
    isPossibleSolution = false;

}

void ExhaustiveSimulation::SetToolParameters(float yaw, float pitch, float roll, float x, float y, float z) {
    dMatrix objPosition;
    dMatrix rotation(dYawMatrix(yaw * 3.1416f / 180.0f) * dPitchMatrix(pitch * 3.1416f / 180.0f) *
                     dRollMatrix(roll * 3.1416f / 180.0f));

    NewtonBodyGetMatrix(m_objBody, &objPosition[0][0]);

    rotation.m_posit = objPosition.m_posit + dVector(offsetX, offsetY, offsetZ);

    NewtonBodySetMatrix(m_toolBody, &rotation[0][0]);
    NewtonWaitForUpdateToFinish(m_world);
}


bool ExhaustiveSimulation::IterateScenario() {
    iterationCount++;

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

void ExhaustiveSimulation::SaveResults() {
    if (isPossibleSolution) {
        cout << "x:" << offsetX << " y:" << offsetY << " z:" << offsetZ;
        cout << " yaw:" << offsetYaw << " pitch:" << offsetPitch << " roll:" << offsetRoll << '\n';
        cout.flush();

        m_output << offsetX << ',' << offsetY << ',' << offsetZ << ',';
        m_output << offsetYaw << ',' << offsetPitch << ',' << offsetRoll << '\n';

        m_output.flush();
    }
}

float ExhaustiveSimulation::GetTimeStep() {
    return 1.0f / MAX_PHYSICS_FPS;
}


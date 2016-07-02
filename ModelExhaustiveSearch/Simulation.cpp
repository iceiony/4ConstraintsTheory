//
// Created by Adrian Ionita on 23/06/2016.
//
#include "Simulation.h"
#include "PhysicsUtils.h"


Simulation::Simulation(const char *const outputFile) : m_world(NewtonCreate()) {

    NewtonSetThreadsCount(m_world, NewtonGetMaxThreadsCount(m_world));

    printf("NewtonDynamicsVersion : %d\n", NewtonWorldGetVersion());
    printf("NewtonThreadCount : %d \n", NewtonGetThreadsCount(m_world));

    //set exact solving
    NewtonSetSolverModel(m_world, 0);

    // clean up all caches the engine have saved
    NewtonInvalidateCache(m_world);

    dTimeTrackerSetThreadName ("mainThread");

    CreateFloor();

    m_output.open(outputFile);
    m_output << "x,y,z,yaw,pitch,roll\n";
}

Simulation::~Simulation() {
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

void Simulation::UpdatePhysics() {
    // update the physics
    if (m_world) {
        NewtonUpdate(m_world, this->GetTimeStep());
    }
}

NewtonBody *Simulation::CreateFloor() {
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

NewtonWorld *Simulation::GetNewtonWorld() {
    return m_world;
}

NewtonBody *Simulation::LoadModel(const char *fileName) {
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

NewtonBody *Simulation::LoadTool(const char *fileName) {
    m_toolBody = this->LoadModel(fileName);

    return m_toolBody;
}

NewtonBody *Simulation::LoadObject(const char *fileName) {
    m_objBody = this->LoadModel(fileName);

    //position lowest point on the floor
    dVector minP, maxP;
    CalculateAABB(m_objBody, minP, maxP);

    m_objInitialPos = dVector(.0f);
    m_objInitialPos.m_y = .0f - minP.m_y;

    return m_objBody;
}

void Simulation::Start() {
    ResetObjPosition();
    SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);
    ReadjustMinMaxLimits();
    SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);
}

void Simulation::ReadjustMinMaxLimits() {//set min/max values for simulation search based on current rotation
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
    minY = std::max(minY, .0f - toolMinP.m_y);
    maxY = (objMaxP.m_y - objPosition.m_y) + (toolPosition.m_y - toolMinP.m_y);

    minZ = (objMinP.m_z - objPosition.m_z) + (toolPosition.m_z - toolMaxP.m_z);
    maxZ = (objMaxP.m_z - objPosition.m_z) + (toolPosition.m_z - toolMinP.m_z);

    minX += positionStep * marginFactor;
    maxX -= positionStep * marginFactor;

    maxY -= positionStep * marginFactor;

    minZ += positionStep * marginFactor;
    maxZ -= positionStep * marginFactor;

    //reset offsets
    offsetX = minX ;
    offsetY = minY ;
    offsetZ = minZ ;
//    offsetX = minX;
//    offsetY = 1.15f;
//    offsetZ = 0.32f;
}

NewtonBody *Simulation::GetFloor() {
    return m_floorBody;
}

bool Simulation::IsFinished() {
    return offsetRoll > maxRoll;
}

void Simulation::ResetObjPosition() {
    dMatrix origin(dGetIdentityMatrix());
    origin.m_posit = m_objInitialPos;
    NewtonBodySetMatrix(m_objBody, &origin[0][0]);
}

void Simulation::NextScenario() {
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

        std::cout << "Yaw increased to " << offsetYaw << '\n';
        std::cout.flush();
    }

    if (offsetYaw > maxYaw) {
        offsetYaw = minYaw;
        offsetPitch += rotationStep;
        isRotationChanged = true;

        std::cout << "Pitch increased to " << offsetPitch << '\n';
        std::cout.flush();
    }

    if (offsetPitch > maxPitch) {
        offsetPitch = minPitch;
        offsetRoll += rotationStep;
        isRotationChanged = true;

        std::cout << 100 * offsetRoll / maxRoll << "%\n";
        std::cout.flush();
    }

    //set tool position to new coordinates
    this->SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);

    if (isRotationChanged) {
        ReadjustMinMaxLimits();
        this->SetToolParameters(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);
    }

    //remove newton_world state after moving the objects
    NewtonInvalidateCache(m_world);

    //remove continuous force application ( i.e. lift force );
    NewtonBodySetForceAndTorqueCallback(m_toolBody, NULL);

    //add small force so invalid collisions to behave chaotically
    NewtonBodySetVelocity(m_toolBody, &dVector(.0f, .1f, .0f)[0]);
    NewtonBodySetOmega(m_toolBody, &dVector(.0f)[0]);

    NewtonBodySetVelocity(m_objBody, &dVector(.0f,.0f,.0f)[0]);
    NewtonBodySetOmega(m_objBody, &dVector(0.0f)[0]);

    //reset scenario tracking
    iterationCount = 0;
    isPossibleSolution = false;

}

void Simulation::SetToolParameters(float yaw, float pitch, float roll, float x, float y, float z) {
    dMatrix objPosition;
    dMatrix rotation(dYawMatrix(yaw * 3.1416f / 180.0f) * dPitchMatrix(pitch * 3.1416f / 180.0f) *
                     dRollMatrix(roll * 3.1416f / 180.0f));

    NewtonBodyGetMatrix(m_objBody, &objPosition[0][0]);

    rotation.m_posit = objPosition.m_posit + dVector(offsetX, offsetY, offsetZ);

    NewtonBodySetMatrix(m_toolBody, &rotation[0][0]);
}


bool Simulation::IterateScenario() {
    iterationCount++;

    //if individual contact points have high impact then scenario failed
    if (!isPossibleSolution && !IsSmallImpact(m_toolBody, m_objBody, 50.0f)) {
        return false;
    }

    if (isPossibleSolution && !IsSmallImpact(m_toolBody,m_objBody,200)){
        isPossibleSolution = false;
        return false;
    }

    //if individual contact points have achieved stability then scenario may succeed
    if (!isPossibleSolution && IsSmallImpact(m_toolBody, m_objBody, 10.0f)) {
        if (!CheckIfBodiesCollide(m_objBody, m_toolBody)) {
            isPossibleSolution = false;
            return false;
        }
        else {
            isPossibleSolution = true;
            NewtonBodySetForceAndTorqueCallback(m_toolBody, MoveTool);
            return true;
        }
    }

    //if object enters sleep state then scenario failed
    if (isPossibleSolution && NewtonBodyGetSleepState(m_objBody) == 1){
        isPossibleSolution = false;
        return false;
    };

    //if objects no velocity after lifting then scenario failed
    if (isPossibleSolution && iterationCount == maxIterationCount) {
        dVector objVelocity;
        NewtonBodyGetVelocity(m_objBody, &objVelocity[0]);

        isPossibleSolution = pow(objVelocity.m_y, 2) >= 0.1f;
        return false;
    }

    return iterationCount < maxIterationCount;
}

void Simulation::SaveResults() {
    if (isPossibleSolution) {
        std::cout << "x:" << offsetX << " y:" << offsetY << " z:" << offsetZ;
        std::cout << " yaw:" << offsetYaw << " pitch:" << offsetPitch << " roll:" << offsetRoll << '\n';
        std::cout.flush();

        m_output << offsetX << ',' << offsetY << ',' << offsetZ << ',';
        m_output << offsetYaw << ',' << offsetPitch << ',' << offsetRoll << '\n';

        m_output.flush();
    }
}

float Simulation::GetTimeStep() {
    return 1.0f / MAX_PHYSICS_FPS;
}


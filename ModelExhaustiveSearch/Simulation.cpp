//
// Created by Adrian Ionita on 23/06/2016.
//
#include "Simulation.h"
#include "PhysicsUtils.h"


Simulation::Simulation(const char * const outputFile) : m_world(NewtonCreate()) {

    NewtonSetThreadsCount(m_world,NewtonGetMaxThreadsCount(m_world));

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

    dMatrix position(dGetIdentityMatrix());

    NewtonBody *modelBody = CreateSimpleBody(m_world, NULL, MASS, position, compound, 0);

    NewtonDestroyCollision(compound);
    NewtonMeshDestroy(convexApproximation);
    NewtonMeshDestroy(mesh);

    return modelBody;
}

NewtonBody *Simulation::LoadObject(const char *fileName) {
    m_objBody = this->LoadModel(fileName);

    m_objInitialPos = dVector(.0f);
    m_objInitialPos.m_y = .0f - GetMinY(m_objBody);

    return m_objBody;
}

NewtonBody *Simulation::LoadTool(const char *fileName) {
    m_toolBody = this->LoadModel(fileName);
    return m_toolBody;
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

    //set new tool coordinates
    offsetX += 0.01;

    if (offsetX > maxX) {
        offsetX = minX;
        offsetY += 0.01;
    }

    if (offsetY > maxY) {
        offsetY = minY;
        offsetZ += 0.01;
    }

    if (offsetZ > maxZ) {
        offsetZ = minZ;
        offsetYaw += 5;
    }

    if (offsetYaw > maxYaw) {
        offsetYaw = minYaw;
        offsetPitch += 5;
    }

    if (offsetPitch > maxPitch) {
        offsetPitch = minPitch;
        offsetRoll += 5;
    }

    //set tool position to new coordinates
    this->SetToolRotation(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);

    offsetY = std::max(.0f - this->GetMinY(m_toolBody), offsetY); //object should not inside the ground

    this->SetToolRotation(offsetYaw, offsetPitch, offsetRoll, offsetX, offsetY, offsetZ);

    //remove newton_world state after moving the objects
    NewtonInvalidateCache(m_world);

    //remove continuous force application ( i.e. lift force );
    NewtonBodySetForceAndTorqueCallback(m_toolBody, NULL);

    //add small force to invalid collisions to behave chaotically
    NewtonBodySetVelocity(m_toolBody, &dVector(.0f, .1f, .0f)[0]);
    NewtonBodySetOmega(m_toolBody,&dVector(.0f)[0]);

    NewtonBodySetVelocity(m_objBody,&dVector(0.0f)[0]);
    NewtonBodySetOmega(m_objBody,&dVector(0.0f)[0]);

    //reset scenario tracking
    iterationCount = 0;
    isPossibleSolution = false;

}

void Simulation::SetToolRotation(float yaw, float pitch, float roll, float x, float y, float z) {
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
    if (!isPossibleSolution && !IsSmallImpact(m_toolBody, m_objBody, 100.0f)) {
//        std::cout << "chaotic ";
        return false;
    }

    //if individual contact points have achieved stability then scenario may succeed
    if (!isPossibleSolution && IsSmallImpact(m_toolBody, m_objBody, 10.0f)) {
        isPossibleSolution = true;
        NewtonBodySetForceAndTorqueCallback(m_toolBody, MoveTool);
        return true;
    }

    //if objects not moving after lifting then scenario failed
    if(isPossibleSolution && iterationCount == maxIterationCount ){
        dVector objVelocity;
        NewtonBodyGetVelocity(m_objBody,&objVelocity[0]);

        isPossibleSolution = pow(objVelocity.m_y,2) >=0.1f ;
//        std::cout << "no contact " << pow(objVelocity.m_y,2);
        return false;
    }

    return iterationCount < maxIterationCount;
}

void Simulation::SaveResults() {
//    if (!isPossibleSolution) exit(0);

    if (isPossibleSolution) {
        std::cout << "x:" << offsetX << " y:" << offsetY << " z:" << offsetZ;
        std::cout << " yaw:" << offsetYaw << " pitch:" << offsetPitch << " roll:" << offsetRoll << '\n';
        std::cout.flush();

        m_output << offsetX << ',' << offsetY << ',' << offsetZ << ',';
        m_output << offsetYaw << ',' << offsetPitch << ',' << offsetRoll << '\n';

        m_output.flush();
    }
}

float Simulation::GetMinY(NewtonBody *body) {
    dVector minP, maxP;
    CalculateAABB(body, minP, maxP);
    return minP.m_y;
}

float Simulation::GetTimeStep() {
    return 1.0f / MAX_PHYSICS_FPS;
}










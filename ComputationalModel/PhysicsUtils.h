/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __PHYSICS_UTIL__
#define __PHYSICS_UTIL__


#define DEMO_GRAVITY  -10.0f

class GraphicsMesh;
class GraphicsEntity;
class GraphicsManager;

NewtonJoint* CheckIfBodiesCollide (NewtonBody* const body0, NewtonBody* const body1);
bool IsSmallPenetration(NewtonBody *const body0, NewtonBody *const body1, float maxPenetration);
bool IsSmallImpact (NewtonBody* const body0, NewtonBody* const body1,float maxForce);
dFloat ForceScalar(dVector force);

void PhysicsApplyGravityForce (const NewtonBody* body, dFloat timestep, int threadIndex);
void MoveTool (const NewtonBody * const body, dFloat timestep, int threadIndex);

void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP);
void CalculateAABB(const NewtonBody *body,dVector &minP ,dVector &maxP);

NewtonBody* CreateSimpleBody (NewtonWorld* const world, void* const userData, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId);
NewtonMesh* LoadMeshFrom3DS(NewtonWorld* const world, const char* const fileName, const dFloat scale);
NewtonMesh* CreateFloorMesh(NewtonWorld* const world);

dFloat RayCast (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam) ;
void UpdateRayCastPosition(dVector startPoints[][VIEW_DIMENSION],dVector endPoints[][VIEW_DIMENSION],dMatrix positionOffset);
#endif
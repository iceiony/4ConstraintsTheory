/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software./Users/juliojerez/Desktop/newton-dynamics/applications/demosSandbox/sdkDemos/toolBox/PhysicsUtils.cpp
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <Util.h>
#include "DemoMesh.h"
#include "DemoEntity.h"
#include "PhysicsUtils.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"


static dFloat RayCastPlacement (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
{
	// if the collision has a parent, the this can be it si a sub shape of a compound collision 
	const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collisionHit);
	if (parent) {
		// you can use this to filter sub collision shapes.  
		dAssert (NewtonCollisionGetSubCollisionHandle (collisionHit));
	}


	dFloat* const paramPtr = (dFloat*)userData;
	if (intersetParam < paramPtr[0]) {
		paramPtr[0] = intersetParam;
	}
	return paramPtr[0];
}


static unsigned RayPrefilter (const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	// if the collision has a parent, the this can be it si a sub shape of a compound collision 
	const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collision);
	if (parent) {
		// you can use this to filter sub collision shapes.  
		dAssert (NewtonCollisionGetSubCollisionHandle (collision));
	}

	return 1;
}

dVector FindFloor (const NewtonWorld* world, const dVector& origin, dFloat dist)
{
	// shot a vertical ray from a high altitude and collect the intersection parameter.
	dVector p0 (origin); 
	dVector p1 (origin - dVector (0.0f, dAbs (dist), 0.0f, 0.0f)); 

	dFloat parameter = 1.2f;
	NewtonWorldRayCast (world, &p0[0], &p1[0], RayCastPlacement, &parameter, RayPrefilter, 0);
	if (parameter < 1.0f) {
		p0 -= dVector (0.0f, dAbs (dist) * parameter, 0.0f, 0.0f);
	}
	return p0;
}

void   MoveTool (const NewtonBody* body, dFloat time, int threadIndex){
	dMatrix position;
	NewtonBodyGetMatrix(body, &position[0][0]);

	if (position.m_posit.m_x > -0.3)
	{
		NewtonBodySetVelocity(body,&dVector(0,1,0)[0]);
		NewtonBodySetOmega(body,&dVector(0,0,0.08)[0]);
	}
	else {
		NewtonBodySetVelocity(body,&dVector(1,0,0)[0]);
		NewtonBodySetOmega(body,&dVector(0,0,0)[0]);
	}
}

// add force and torque to rigid body
void  PhysicsApplyGravityForce (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);
//mass*= 0.0f;

	dVector force (dVector (0.0f, 1.0f, 0.0f).Scale (mass * DEMO_GRAVITY));
	NewtonBodySetForce (body, &force.m_x);
}

NewtonBody* CreateSimpleBody (NewtonWorld* const world, void* const userData, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId)
{

	// calculate the moment of inertia and the relative center of mass of the solid
	//	dVector origin;
	//	dVector inertia;
	//	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	
	//	dFloat Ixx = mass * inertia[0];
	//	dFloat Iyy = mass * inertia[1];
	//	dFloat Izz = mass * inertia[2];

	//create the rigid body
	NewtonBody* const rigidBody = NewtonCreateDynamicBody (world, collision, &matrix[0][0]);

	// set the correct center of gravity for this body (these function are for legacy)
	//	NewtonBodySetCentreOfMass (rigidBody, &origin[0]);
	//	NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);

	// use a more convenient function for setting mass and inertia matrix
	NewtonBodySetMassProperties (rigidBody, mass, collision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, userData);

	// assign the wood id
	NewtonBodySetMaterialGroupID (rigidBody, materialId);

	//  set continuous collision mode
	//	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// set the matrix for both the rigid body and the graphic body
	//NewtonBodySetMatrix (rigidBody, &matrix[0][0]);
	//PhysicsSetTransform (rigidBody, &matrix[0][0], 0);

	//dVector xxx (0, -9.8f * mass, 0.0f, 0.0f);
	//NewtonBodySetForce (rigidBody, &xxx[0]);

	// force the body to be active of inactive
	//	NewtonBodySetAutoSleep (rigidBody, sleepMode);
	return rigidBody;
}

NewtonBody* CreateSimpleSolid (DemoEntityManager* const scene, DemoMesh* const mesh, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId)
{
	dAssert (collision);

	// add an new entity to the world
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	scene->Append (entity);
	if (mesh) {
		entity->SetMesh(mesh, dGetIdentityMatrix());
	}
	return CreateSimpleBody (scene->GetNewton(), entity, mass, matrix, collision, materialId);
}


void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP)
{
	dFloat skinThickness = NewtonCollisionGetSkinThickness (collision) * 0.125f;
	for (int i = 0; i < 3; i ++) {
		dVector support(0.0f);
		dVector dir (0.0f);
		dir[i] = 1.0f;

		dVector localDir (matrix.UnrotateVector (dir));
		NewtonCollisionSupportVertex (collision, &localDir[0], &support[0]);
		support = matrix.TransformVector (support);
		maxP[i] = support[i] - skinThickness;  

		localDir = localDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &localDir[0], &support[0]);
		support = matrix.TransformVector (support);
		minP[i] = support[i] + skinThickness;  
	}
}

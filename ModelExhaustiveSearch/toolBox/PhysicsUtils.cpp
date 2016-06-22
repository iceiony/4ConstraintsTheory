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
void  PhysicsApplyGravityForce (const NewtonBody* const body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);

	dVector force (dVector (0.0f, 1.0f, 0.0f).Scale (mass * DEMO_GRAVITY));
	NewtonBodySetForce (body, &force.m_x);
}

NewtonBody* CreateSimpleBody (NewtonWorld* const world, void* const userData, dFloat mass, const dMatrix& position, NewtonCollision* const collision, int materialId)
{

	//create the rigid body
	NewtonBody* const rigidBody = NewtonCreateDynamicBody (world, collision, &position[0][0]);

	// use a more convenient function for setting mass and inertia matrix
	NewtonBodySetMassProperties (rigidBody, mass, collision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, userData);

	// assign the wood id
	NewtonBodySetMaterialGroupID (rigidBody, materialId);

	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	return rigidBody;
}

NewtonBody* CreateSimpleSolid (DemoEntityManager* const scene, DemoMesh* const mesh, dFloat mass, const dMatrix& position, NewtonCollision* const collision, int materialId)
{
	dAssert (collision);

	// add an new entity to the world
	DemoEntity* const entity = new DemoEntity(position, NULL);
	scene->Append (entity);
	if (mesh) {
		entity->SetMesh(mesh, dGetIdentityMatrix());
	}
	return CreateSimpleBody (scene->GetNewton(), entity, mass, position, collision, materialId);
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

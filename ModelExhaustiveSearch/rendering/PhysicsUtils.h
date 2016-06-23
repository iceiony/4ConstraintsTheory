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

enum PrimitiveType
{
	_NULL_PRIMITIVE,
	_SPHERE_PRIMITIVE,
	_BOX_PRIMITIVE,
	_CAPSULE_PRIMITIVE,
	_CYLINDER_PRIMITIVE,
	_CONE_PRIMITIVE,
	_CHAMFER_CYLINDER_PRIMITIVE,
	_RANDOM_CONVEX_HULL_PRIMITIVE,
	_REGULAR_CONVEX_HULL_PRIMITIVE,
	_COMPOUND_CONVEX_CRUZ_PRIMITIVE,
};

class GraphicsMesh;
class GraphicsEntity;
class GraphicsManager;

dVector FindFloor (const NewtonWorld* world, const dVector& origin, dFloat dist);

void PhysicsApplyGravityForce (const NewtonBody* body, dFloat timestep, int threadIndex);
void MoveTool (const NewtonBody * const body, dFloat timestep, int threadIndex);

void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP);

NewtonBody* CreateSimpleBody (NewtonWorld* const world, void* const userData, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId);
NewtonMesh* LoadMeshFrom3DS(NewtonWorld* const world, const char* const fileName, const dFloat scale);
NewtonMesh* CreateFloorMesh(NewtonWorld* const world);
#endif
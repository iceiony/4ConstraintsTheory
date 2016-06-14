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


#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"

static bool ReportProgress (dFloat normalizedProgressPercent, void* const userData)
{
	return true; 
}


static NewtonBody* CreateConvexAproximation (const char* const name, DemoEntityManager* const scene, const dVector& origin ,dFloat mass, const char* const texture)
{
	char fileName[2048];

	GetWorkingFileName (name, fileName);
	NewtonWorld* const world = scene->GetNewton();
	NewtonMesh* const mesh = NewtonMeshLoadOFF(world, fileName);

	dMatrix rotate (dPitchMatrix(-90.0f * 3.1416f / 180.0f));
	NewtonMeshApplyTransform(mesh,&rotate[0][0]);

	//NewtonMesh* const newtonMesh = NewtonMeshSimplify(mesh, 500, ReportProgress);

	// create a convex approximation form the original mesh, 32 convex max and no more than 100 vertex convex hulls
//	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 32, 100, ReportProgress, scene);
//	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 256, 100, ReportProgress, scene);
	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.00001f, 0.0f, 256, 100, ReportProgress, scene);

//	NewtonMesh* const convexApproximation = mesh;

	// create a compound collision by creation a convex hull of each segment of the source mesh 
	NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (world, convexApproximation, 0.001f, 0, 0);

	// test collision mode
//	NewtonCollisionSetCollisonMode(compound, 0);

	// make a visual Mesh
	int tex = LoadTexture(texture);
	NewtonMeshApplyBoxMapping(mesh, tex, tex, tex);
	DemoMesh* const visualMesh = new DemoMesh (mesh);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
//	matrix.RotateVector(dVector(0,90,0));

	NewtonBody* body = CreateSimpleSolid (scene, visualMesh, mass, matrix, compound, 0);


	visualMesh->Release();

	NewtonDestroyCollision(compound);
	NewtonMeshDestroy (convexApproximation);

	return body;
}



void SimpleConvexApproximation (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
//	NewtonBody* const floor = CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	dMatrix originMatrix;
	NewtonBodyGetMatrix(floor, &originMatrix[0][0]);

	dMatrix camMatrix (dRollMatrix(0.0f * 3.1416f /180.0f) * dYawMatrix(-90.0f * 3.1416f /180.0f));
	dQuaternion rot (camMatrix);
	dVector origin (originMatrix.m_posit);
	dFloat hight = 0.0f;
	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, hight, origin .m_z, 0.0f), hight * 2);
	

	dVector location (origin);
	location.m_x += 0.0f;
	location.m_z += 0.0f;
	location.m_y += 1.0f;

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());


	// convex approximate some file meshes 

	NewtonBody* objBody = CreateConvexAproximation ("obj51.off", scene, location, 10.0f, "wood_1.tga");
	NewtonBody* toolBody = CreateConvexAproximation ("obj52.off", scene, location + dVector(-3,0.2f,0.02f), 10.0f, "sky.tga");


    //rotate tool body in proper position
    dMatrix currentPos ;
    dMatrix rotated (dPitchMatrix(180.0f * 3.1416f / 180.0f)) ;
    NewtonBodyGetMatrix(toolBody,&currentPos[0][0]);
    rotated.m_posit = currentPos.m_posit;
    NewtonBodySetMatrix(toolBody,&rotated[0][0]);

	NewtonBodySetForceAndTorqueCallback(toolBody,MoveTool);

	dVector velocity(1,0,0);
	NewtonBodySetVelocity(toolBody,&velocity[0]);

	dVector size (0.5f, 0.5f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	

	origin.m_y += 1.0f;
	scene->SetCameraMatrix(rot, origin-dVector(0,0,5));
}



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

#include <iostream>
#include <vector>
#include <Util.h>
#include <lib3ds/file.h>
#include <lib3ds/mesh.h>
#include <lib3ds/vector.h>
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

static bool ReportProgress (dFloat normalizedProgressPercent, void* const userData)
{
	return true; 
}


static NewtonBody* CreateFloor (DemoEntityManager* const scene)
{
	const dVector origin(0.0f,0.0f,0.0f);
	const dVector scale(4.0f,0.01f,4.0f);
	dFloat mass = 0.0f;

	// the vertex array, vertices's has for values, x, y, z, w
	// w is use as a id to have multiple copy of the same very, like for example mesh that share more than two edges.
	// in most case w can be set to 0.0
	static dFloat points[] = {
			-1.0f, -1.0f, -1.0f,
			-1.0f, -1.0f,  1.0f,
			-1.0f,  1.0f,  1.0f,
			-1.0f,  1.0f, -1.0f,
			 1.0f, -1.0f, -1.0f,
			 1.0f, -1.0f,  1.0f,
			 1.0f,  1.0f,  1.0f,
			 1.0f,  1.0f, -1.0f,
	};

	// the vertex index list is an array of all the face, in any order, the can be convex or concave,
	// and has and variable umber of indices
	static std::vector<std::vector<int>> faces = {
			{2,3,0,1},
			{5,2,1},
			{6,2,5},
			{5,1,0,4},
			{2,7,3},
			{6,7,2},
			{3,4,0},
			{7,4,3},
			{7,5,4},
			{6,5,7}
	};


	dVector scaled[8] ;
	for (int i = 0; i < 8; i ++) {
		scaled[i] = scale.CompProduct(dVector (&points[i * 3]));
	}

	// now we create and empty mesh
	NewtonMesh* const newtonMesh = NewtonMeshCreate (scene->GetNewton());

	NewtonMeshBeginFace(newtonMesh);
	for(auto face = faces.begin(); face != faces.end() ; face++ ){

		dFloat vertices[12];
		for(unsigned idx=0;idx<face->size();idx++){
			vertices[3*idx]   = scaled[face->at(idx)].m_x;
			vertices[3*idx+1] = scaled[face->at(idx)].m_y;
			vertices[3*idx+2] = scaled[face->at(idx)].m_z;
		}

		NewtonMeshAddFace(newtonMesh,face->size(),vertices,sizeof(dFloat)*3,0);
	}
	NewtonMeshEndFace(newtonMesh);

	// now we can use this mesh for lot of stuff, we can apply UV, we can decompose into convex,
	NewtonCollision* const collision = NewtonCreateConvexHullFromMesh(scene->GetNewton(), newtonMesh, 0.001f, 0);

	// for now we will simple make simple Box,  make a visual Mesh
	DemoMesh* const visualMesh = new DemoMesh (newtonMesh);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	NewtonBody* const body = CreateSimpleSolid (scene, visualMesh, mass, matrix, collision, 0);

	visualMesh->Release();
	NewtonDestroyCollision(collision);
	NewtonMeshDestroy (newtonMesh);

	return body;
}

static NewtonMesh* LoadMeshFrom3DS(NewtonWorld* const world, const char* const fileName, const dFloat const scale){
	NewtonMesh* meshNewton = NewtonMeshCreate(world);

	Lib3dsFile* modelFile = lib3ds_file_load(fileName);
	if(!modelFile){
		std::cerr << "Failed to load file ( not 3DS format ? ) : " << fileName;
		return meshNewton;
	}

	NewtonMeshBeginFace(meshNewton);

	Lib3dsMesh* mesh;
	for(mesh = modelFile->meshes; mesh != 0 ;mesh = mesh->next)
	{
		unsigned p;
		for(p = 0; p < mesh->faces ;p++ ){
			Lib3dsFace* face = &mesh->faceL[p];

			dFloat vertices[9]; // 3 points x 3 dimensions
			for(int idx = 0;idx < 3 ;idx++){
				vertices[3*idx]   = mesh->pointL[face->points[idx]].pos[0] * scale;
				vertices[3*idx+1] = mesh->pointL[face->points[idx]].pos[1] * scale;
				vertices[3*idx+2] = mesh->pointL[face->points[idx]].pos[2] * scale;
			}

			NewtonMeshAddFace(meshNewton,3,vertices,sizeof(dFloat)*3,0);
		}
	}

	NewtonMeshEndFace(meshNewton);

	dMatrix rotate (dPitchMatrix(-90.0f * 3.1416f / 180.0f));
	NewtonMeshApplyTransform(meshNewton,&rotate[0][0]);

 	return meshNewton;
}

static NewtonBody* CreateConvexApproximation (const char* const fileName, DemoEntityManager* const scene, const dVector& origin ,dFloat mass)
{
	NewtonWorld* const world = scene->GetNewton();
	NewtonMesh* const mesh = LoadMeshFrom3DS(world, fileName, 0.008);

	//NewtonMesh* const newtonMesh = NewtonMeshSimplify(mesh, 500, ReportProgress);

	// create a convex approximation form the original mesh, 32 convex max and no more than 100 vertex convex hulls
//	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 32, 100, ReportProgress, scene);
//	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 256, 100, ReportProgress, scene);
	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.00001f, 0.0f, 256, 100, ReportProgress, scene);
//	NewtonMesh* const convexApproximation = mesh;

	// create a compound collision by creation a convex hull of each segment of the source mesh
	NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (world, convexApproximation, 0.001f, 0, 0);

	// make a visual Mesh
	DemoMesh* const visualMesh = new DemoMesh (mesh);

	dMatrix position (dGetIdentityMatrix());
	position.m_posit = origin;

	NewtonBody* body = CreateSimpleSolid (scene, visualMesh, mass, position, compound, 0);

	visualMesh->Release();

	NewtonDestroyCollision(compound);
	NewtonMeshDestroy(convexApproximation);
	NewtonMeshDestroy(mesh);

	return body;
}



void SimpleConvexApproximation (DemoEntityManager* const scene)
{
	// load the scene from a ngd file format
	NewtonBody* floor = CreateFloor(scene);

	dMatrix originMatrix;
	NewtonBodyGetMatrix(floor, &originMatrix[0][0]);

	dMatrix camMatrix (dRollMatrix(0.0f * 3.1416f /180.0f) * dYawMatrix(0.0f * 3.1416f /180.0f));
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
	NewtonBody* objBody = CreateConvexApproximation ("obj51.3ds", scene, location, 10.0f);
	NewtonBody* toolBody = CreateConvexApproximation ("obj52.3ds", scene, location + dVector(-3,0.15f,0.326f), 10.0f);


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
	scene->SetCameraMatrix(rot, origin-dVector(5,0,0));

}



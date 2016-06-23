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
#include "GraphicsEntity.h"
#include "PhysicsUtils.h"
#include <lib3ds/file.h>
#include <lib3ds/mesh.h>
#include <vector>

NewtonMesh *LoadMeshFrom3DS(NewtonWorld *const world, const char *const fileName, const dFloat scale) {
    NewtonMesh *meshNewton = NewtonMeshCreate(world);

    Lib3dsFile *modelFile = lib3ds_file_load(fileName);
    if (!modelFile) {
        std::cerr << "Failed to load file ( not 3DS format ? ) : " << fileName;
        return meshNewton;
    }

    NewtonMeshBeginFace(meshNewton);

    Lib3dsMesh *mesh;
    for (mesh = modelFile->meshes; mesh != 0; mesh = mesh->next) {
        unsigned p;
        for (p = 0; p < mesh->faces; p++) {
            Lib3dsFace *face = &mesh->faceL[p];

            dFloat vertices[9]; // 3 points x 3 dimensions
            for (int idx = 0; idx < 3; idx++) {
                vertices[3 * idx] = mesh->pointL[face->points[idx]].pos[0] * scale;
                vertices[3 * idx + 1] = mesh->pointL[face->points[idx]].pos[1] * scale;
                vertices[3 * idx + 2] = mesh->pointL[face->points[idx]].pos[2] * scale;
            }

            NewtonMeshAddFace(meshNewton, 3, vertices, sizeof(dFloat) * 3, 0);
        }
    }

    NewtonMeshEndFace(meshNewton);

    //for some reason the mesh objects are rotated sideways at a 90 degree angle
    dMatrix rotate(dPitchMatrix(-90.0f * 3.1416f / 180.0f));
    NewtonMeshApplyTransform(meshNewton, &rotate[0][0]);

    return meshNewton;
}

static dFloat RayCastPlacement(const NewtonBody *const body, const NewtonCollision *const collisionHit,
                               const dFloat *const contact, const dFloat *const normal, dLong collisionID,
                               void *const userData, dFloat intersetParam) {
    // if the collision has a parent, the this can be it si a sub shape of a compound collision
    const NewtonCollision *const parent = NewtonCollisionGetParentInstance(collisionHit);
    if (parent) {
        // you can use this to filter sub collision shapes.
        dAssert (NewtonCollisionGetSubCollisionHandle(collisionHit));
    }


    dFloat *const paramPtr = (dFloat *) userData;
    if (intersetParam < paramPtr[0]) {
        paramPtr[0] = intersetParam;
    }
    return paramPtr[0];
}


static unsigned RayPrefilter(const NewtonBody *const body, const NewtonCollision *const collision,
                             void *const userData) {
    // if the collision has a parent, the this can be it si a sub shape of a compound collision
    const NewtonCollision *const parent = NewtonCollisionGetParentInstance(collision);
    if (parent) {
        // you can use this to filter sub collision shapes.
        dAssert (NewtonCollisionGetSubCollisionHandle(collision));
    }

    return 1;
}

dVector FindFloor(const NewtonWorld *world, const dVector &origin, dFloat dist) {
    // shot a vertical ray from a high altitude and collect the intersection parameter.
    dVector p0(origin);
    dVector p1(origin - dVector(0.0f, dAbs(dist), 0.0f, 0.0f));

    dFloat parameter = 1.2f;
    NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastPlacement, &parameter, RayPrefilter, 0);
    if (parameter < 1.0f) {
        p0 -= dVector(0.0f, dAbs(dist) * parameter, 0.0f, 0.0f);
    }
    return p0;
}

void MoveTool(const NewtonBody * const body, dFloat time, int threadIndex) {
    dMatrix position;
    NewtonBodyGetMatrix(body, &position[0][0]);

    if (position.m_posit.m_x > -0.3) {
        NewtonBodySetVelocity(body, &dVector(0, 1, 0)[0]);
        NewtonBodySetOmega(body, &dVector(0, 0, 0.08)[0]);
    }
    else {
        NewtonBodySetVelocity(body, &dVector(1, 0, 0)[0]);
        NewtonBodySetOmega(body, &dVector(0, 0, 0)[0]);
    }
}

// add force and torque to rigid body
void PhysicsApplyGravityForce(const NewtonBody *const body, dFloat timestep, int threadIndex) {
    dFloat Ixx;
    dFloat Iyy;
    dFloat Izz;
    dFloat mass;

    NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

    dVector force(dVector(0.0f, 1.0f, 0.0f).Scale(mass * DEMO_GRAVITY));
    NewtonBodySetForce(body, &force.m_x);
}

NewtonBody *CreateSimpleBody(NewtonWorld *const world, void *const userData, dFloat mass, const dMatrix &position,
                             NewtonCollision *const collision, int materialId) {

    //create the rigid body
    NewtonBody *const rigidBody = NewtonCreateDynamicBody(world, collision, &position[0][0]);

    // use a more convenient function for setting mass and inertia matrix
    NewtonBodySetMassProperties(rigidBody, mass, collision);

    // save the pointer to the graphic object with the body.
    if (userData)
        NewtonBodySetUserData(rigidBody, userData);

    // assign the wood id
    NewtonBodySetMaterialGroupID(rigidBody, materialId);

    // set the transform call back function
    NewtonBodySetTransformCallback(rigidBody, GraphicsEntity::TransformCallback);

    // set the force and torque call back function
    NewtonBodySetForceAndTorqueCallback(rigidBody, PhysicsApplyGravityForce);

    return rigidBody;
}

void CalculateAABB(const NewtonCollision *const collision, const dMatrix &matrix, dVector &minP, dVector &maxP) {
    dFloat skinThickness = NewtonCollisionGetSkinThickness(collision) * 0.125f;
    for (int i = 0; i < 3; i++) {
        dVector support(0.0f);
        dVector dir(0.0f);
        dir[i] = 1.0f;

        dVector localDir(matrix.UnrotateVector(dir));
        NewtonCollisionSupportVertex(collision, &localDir[0], &support[0]);
        support = matrix.TransformVector(support);
        maxP[i] = support[i] - skinThickness;

        localDir = localDir.Scale(-1.0f);
        NewtonCollisionSupportVertex(collision, &localDir[0], &support[0]);
        support = matrix.TransformVector(support);
        minP[i] = support[i] + skinThickness;
    }
}

NewtonMesh *CreateFloorMesh(NewtonWorld *const world) {
    const dVector scale(4.0f, 0.01f, 4.0f);

// the vertex array, vertices's has for values, x, y, z, w
// w is use as a id to have multiple copy of the same very, like for example mesh that share more than two edges.
// in most case w can be set to 0.0
    static dFloat points[] = {
            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, -1.0f,
    };

// the vertex index list is an array of all the face, in any order, the can be convex or concave,
// and has and variable umber of indices
    static std::vector<std::vector<int>> faces = {
            {2, 3, 0, 1},
            {5, 2, 1},
            {6, 2, 5},
            {5, 1, 0, 4},
            {2, 7, 3},
            {6, 7, 2},
            {3, 4, 0},
            {7, 4, 3},
            {7, 5, 4},
            {6, 5, 7}
    };


    dVector scaled[8];
    for (int i = 0; i < 8; i++) {
        scaled[i] = scale.CompProduct(dVector(&points[i * 3]));
    }

// now we create and empty mesh
    NewtonMesh *const newtonMesh = NewtonMeshCreate(world);

    NewtonMeshBeginFace(newtonMesh);
    for (auto face = faces.begin(); face != faces.end(); face++) {

        dFloat vertices[12];
        for (unsigned idx = 0; idx < face->size(); idx++) {
            vertices[3 * idx] = scaled[face->at(idx)].m_x;
            vertices[3 * idx + 1] = scaled[face->at(idx)].m_y;
            vertices[3 * idx + 2] = scaled[face->at(idx)].m_z;
        }

        NewtonMeshAddFace(newtonMesh, face->size(), vertices, sizeof(dFloat) * 3, 0);
    }
    NewtonMeshEndFace(newtonMesh);

    return newtonMesh;
}

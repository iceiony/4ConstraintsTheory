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
#include "PhysicsUtils.h"
#include <lib3ds/file.h>
#include <lib3ds/mesh.h>
#include <vector>
#include <math.h>

// return the collision joint, if the body collide
NewtonJoint *CheckIfBodiesCollide(NewtonBody *const body0, NewtonBody *const body1) {
    for (NewtonJoint *joint = NewtonBodyGetFirstContactJoint(body0); joint; joint = NewtonBodyGetNextContactJoint(body0, joint)) {
        if (NewtonJointIsActive(joint) && (NewtonJointGetBody0(joint) == body1 || NewtonJointGetBody1(joint) == body1)) {
            return joint;
        }
    }
    return NULL;
}

dFloat ForceScalar(dVector force) {
    return (dFloat) sqrt(pow(force.m_x, 2) + pow(force.m_y, 2) + pow(force.m_z, 2));
}

bool IsSmallPenetration(NewtonBody *const body0, NewtonBody *const body1, float maxPenetration) {
    bool noJoint = true;
    for (NewtonJoint *joint = NewtonBodyGetFirstContactJoint(body0); joint; joint = NewtonBodyGetNextContactJoint(body0, joint))
    {
        if (NewtonJointIsActive(joint) && (NewtonJointGetBody0(joint) == body1 || NewtonJointGetBody1(joint) == body1))
        {
            noJoint = false;
            for (void *contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact))
            {
                NewtonMaterial *const material = NewtonContactGetMaterial(contact);
                if (NewtonMaterialGetContactPenetration(material) > maxPenetration)
                {
//                    std::cout << "Joint penetration :" << NewtonMaterialGetContactPenetration(material);
                    return false;
                }
            }
        }
    }

    return true;
}

bool IsSmallImpact(NewtonBody *const body0, NewtonBody *const body1, float maxForce) {
    for (NewtonJoint *joint = NewtonBodyGetFirstContactJoint(body0); joint; joint = NewtonBodyGetNextContactJoint(body0, joint)) {
        if (NewtonJointIsActive(joint) && (NewtonJointGetBody0(joint) == body0 || NewtonJointGetBody0(joint) == body1)) {
            for (void *contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
                dVector point(0.0f);
                dVector normal(0.0f);
                dVector contactForce(0.0f);
                NewtonMaterial *const material = NewtonContactGetMaterial(contact);
                NewtonMaterialGetContactPositionAndNormal(material, body0, &point.m_x, &normal.m_x);
                NewtonMaterialGetContactForce(material, body0, &contactForce[0]);

                if (ForceScalar(contactForce) >= maxForce) {
                    return false;
                }

            }
            break;
        }
    }
    return true;
}

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



void MoveTool(const NewtonBody *const body, dFloat time, int threadIndex) {
    NewtonBodySetVelocity(body, &dVector(0, 1, 0)[0]);

    dVector omega;
    NewtonBodyGetOmega(body, &omega[0]);

    omega = dVector(0, 0, 0) - omega;
    NewtonBodySetOmega(body, &omega[0]);
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

void CalculateAABB(const NewtonBody *body, dVector &minP, dVector &maxP) {
    dMatrix matrix;

    NewtonCollision *const collision = NewtonBodyGetCollision(body);
    NewtonBodyGetMatrix(body, &matrix[0][0]);
    NewtonCollisionCalculateAABB(collision, &matrix[0][0], &minP[0], &maxP[0]);
    CalculateAABB(collision, matrix, minP, maxP);
}

NewtonMesh *CreateFloorMesh(NewtonWorld *const world) {
    const dVector scale(4.0f, 0.0001f, 4.0f);

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

dFloat RayCast (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam) {
    dFloat *const paramPtr = (dFloat *) userData;
    if (intersetParam < paramPtr[0]) {
        paramPtr[0] = intersetParam;
    }
    return paramPtr[0];
}

void UpdateRayCastPosition(dVector startPoints[][VIEW_DIMENSION]
        ,dVector endPoints[][VIEW_DIMENSION]
        ,dMatrix positionOffset) {

//    dVector position = positionOffset.m_front;
//    std::cout << position.m_x << ' ' << position.m_y << ' ' << position.m_z  << "\n\r";
//    position = positionOffset.m_posit;

    float mid = (float) VIEW_DIMENSION / 2.0f;
    for(int i=0;i<VIEW_DIMENSION;i++){
        for(int j=0;j<VIEW_DIMENSION;j++){
            float rotationFactor = positionOffset.m_posit.m_y * 1.57;

            startPoints[i][j].m_x = (i-mid)*CAST_STEP + positionOffset.m_posit.m_x;
            startPoints[i][j].m_y = positionOffset.m_posit.m_y;
            startPoints[i][j].m_z = (j-mid)*CAST_STEP + positionOffset.m_posit.m_z;

            endPoints[i][j].m_x = (i - mid)*CAST_STEP + positionOffset.m_posit.m_x + rotationFactor * positionOffset.m_front.m_x;
            endPoints[i][j].m_y = -0.1f;
            endPoints[i][j].m_z = (j - mid)*CAST_STEP + positionOffset.m_posit.m_z + rotationFactor * positionOffset.m_front.m_z;
        }
    }
}

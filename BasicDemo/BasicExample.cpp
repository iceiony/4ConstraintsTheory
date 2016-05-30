/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
    btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

    m_dynamicsWorld->setGravity(btVector3(0,-20.,0));

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
    createGround();

    //load tool and object
    toolBody = loadMeshObject("obj52.obj", toolDefaultLocation, toolMass, 3);
    objBody = loadMeshObject("obj51.obj", objDefaultLocation, objMass, 2.92);

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

    //initialise tool fitting search
    angle = angleMin;
    yOffset = yMin;
    zOffset = zMin;
    finished = false;

    toolBody->setGravity(btVector3(0,0,0));
    toolBody->setCollisionFlags(toolBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    toolBody->setActivationState(DISABLE_DEACTIVATION);

//    toolBody->setDamping();

//    toolBody->setDamping(toolBody->getLinearDamping()+5, toolBody->getAngularDamping()+5);
//    objBody->setDamping(objBody->getLinearDamping()+5, objBody->getAngularDamping()+5);
//    toolBody->setFriction(toolBody->getFriction()*5);
//    objBody->setFriction(objBody->getFriction()*5);
    printf("Friction : %f\n",toolBody->getFriction());
    printf("Linear Damping : %f\n",toolBody->getLinearDamping());
    printf("Angular Damping : %f\n",toolBody->getAngularDamping());
    nextScenario();
}

btRigidBody * BasicExample::loadMeshObject(const char *fileName, const btVector3 &position, const btScalar &mass, const float scaleFactor) {
    GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(fileName, "");
    printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);

    b3AlignedObjectArray<btVector3> verts;
    for (int i = 0 ; i < glmesh->m_numvertices ; i++){
        float *cords = glmesh->m_vertices->at(i).xyzw;
        verts.push_back(btVector3(cords[0],cords[1],cords[2]));
    }

    btTriangleMesh *vertexMesh = new btTriangleMesh();
    for (int i =0 ; i< glmesh->m_numIndices ; i += 3){
        int idx1 = glmesh->m_indices->at(i);
        int idx2 = glmesh->m_indices->at(i+1);
        int idx3 = glmesh->m_indices->at(i+2);

        vertexMesh->addTriangle(verts.at(idx1),verts.at(idx2),verts.at(idx3));
    }

    btGImpactMeshShape* shape = new btGImpactMeshShape(vertexMesh);

    btVector3 scaling(scaleFactor,scaleFactor,scaleFactor);
    btVector3 color(1.,0.3,0.3);

    shape->setLocalScaling(scaling);
//    shape->setMargin(-0.0001);
    shape->updateBound();

    m_collisionShapes.push_back(shape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    startTransform.setOrigin(position);
    btRigidBody* body = createRigidBody(mass, startTransform, shape);

    if (m_guiHelper->getRenderInterface()) {
        int shapeId = m_guiHelper->getRenderInterface()->registerShape(&glmesh->m_vertices->at(0).xyzw[0],
                                                                       glmesh->m_numvertices,
                                                                       &glmesh->m_indices->at(0),
                                                                       glmesh->m_numIndices,
                                                                       B3_GL_TRIANGLES, -1);
        shape->setUserIndex(shapeId);
        int renderInstance = m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId, position,
                                                                                         startTransform.getRotation(),
                                                                                         color, scaling);
        body->setUserIndex(renderInstance);
    }

    delete(glmesh);
    verts.clear();

    return body;
}

void BasicExample::createGround() {
    btBoxShape* groundShape = createBoxShape(btVector3(btScalar(10.), btScalar(1.), btScalar(10.)));

    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -1., 0));

    btScalar mass(0.);

    btRigidBody* body= createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));

    if (m_guiHelper->getRenderInterface()) {
        btVector3 color(1, 0.7, 0.7);
        int graphicsShapeId = groundShape->getUserIndex();
        if (graphicsShapeId >= 0) {

            btVector3 localScaling(1, 1, 1);
            int graphicsInstanceId = m_guiHelper->getRenderInterface()->registerGraphicsInstance(graphicsShapeId,
                                                                                                 groundTransform.getOrigin(),
                                                                                                 groundTransform.getRotation(),
                                                                                                 color, localScaling);
            body->setUserIndex(graphicsInstanceId);
        }
    }
}

void BasicExample::stepSimulation(float deltaTime){

    bool isResetRequired = toolBody->getCenterOfMassPosition().getY() > 10;

    if (isResetRequired) {
        bool isSolution = objBody->getCenterOfMassPosition().getY() > 10;
        if(isSolution) printf("[INFO] Solution Found y:%f z:%f angle:%f \n",yOffset,zOffset,angle);
        nextScenario();
    }
    else {
        btTransform transform;
        toolBody->getMotionState()->getWorldTransform(transform);
        if( transform.getOrigin().getX() > -0.8 ) transform.getOrigin() += btVector3(-0.015,0,0);
        else transform.getOrigin() += btVector3(0,0.015,0);
        toolBody->getMotionState()->setWorldTransform(transform);
    }

    CommonRigidBodyBase::stepSimulation(deltaTime);
}

void BasicExample::nextScenario(){
    objBody->setLinearVelocity(btVector3(0,0,0));
    objBody->setAngularVelocity(btVector3(0,0,0));
    objBody->setInterpolationAngularVelocity(btVector3(0,0,0));
    objBody->setInterpolationLinearVelocity(btVector3(0,0,0));
    objBody->clearForces();

    //reset object position

    btTransform transform = objBody->getCenterOfMassTransform();
    transform.setOrigin(objDefaultLocation);
    transform.setRotation(btQuaternion(3.1416,0,0));
    objBody->setCenterOfMassTransform(transform);

    //configure new scenario

    transform = toolBody->getCenterOfMassTransform();

    btVector3 toolLocation(toolDefaultLocation);
//    toolLocation.setY( toolLocation.getY() + nextY() );
//    toolLocation.setZ( toolLocation.getZ() + nextZ() );
    transform.setOrigin(toolLocation);

    transform.setRotation(btQuaternion(-angle,0,0));

    toolBody->setCenterOfMassTransform(transform);

    transform.setRotation(nextRotation());
    toolBody->setCenterOfMassTransform(transform);
    toolBody->getMotionState()->setWorldTransform(transform);
}


void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
    m_dynamicsWorld->debugDrawWorld();

}

const btScalar &BasicExample::nextZ() {
    if (zOffset > zMax ) {
        finished = true;
    }
    return zOffset;
}

const btScalar &BasicExample::nextY() {
    if (yOffset > yMax) {
        yOffset = yMin;
        zOffset += zStep;
    };
    return yOffset;
}

btQuaternion BasicExample::nextRotation() {
    angle += angleStep;
    if(angle>angleMax){
        angle = angleMin;
        yOffset += yStep;
    }
    return btQuaternion(0,0,3.1416);
}

bool BasicExample::isFinished(){
    return finished;
}

CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)




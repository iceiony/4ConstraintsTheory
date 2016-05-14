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

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 30;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

    void createGround();

    void loadMeshObject(const char *fileName, const btVector3 &scaling, const btVector3 &color, btScalar mass);
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,-30.,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	///create a few basic rigid bodies
    createGround();

    //load obj mesh
	const char* fileName = "lego_tool1.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";

    btVector3 scaling(1.,1.,1.);
    btVector3 color(1.,0.3,0.3);
    btScalar	mass(3.f);

    loadMeshObject(fileName, scaling, color, mass);

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BasicExample::loadMeshObject(const char *fileName, const btVector3 &scaling, const btVector3 &color,
                                  btScalar mass) {
    GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(fileName, "");
    printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);

    const GLInstanceVertex& v = glmesh->m_vertices->at(0);
    btConvexHullShape* shape = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])), glmesh->m_numvertices, sizeof(GLInstanceVertex));

    shape->setLocalScaling((scaling));
    shape->optimizeConvexHull();
    shape->initializePolyhedralFeatures();

    //shape->setMargin(0.001);
    m_collisionShapes.push_back(shape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    btVector3 position(0,10,0);
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
            //	btAssert(graphicsShapeId >= 0);
            //the graphics shape is already scaled
            btVector3 localScaling(1, 1, 1);
            int graphicsInstanceId = m_guiHelper->getRenderInterface()->registerGraphicsInstance(graphicsShapeId,
                                                                                                 groundTransform.getOrigin(),
                                                                                                 groundTransform.getRotation(),
                                                                                                 color, localScaling);
            body->setUserIndex(graphicsInstanceId);
        }
    }
}


void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
	
}


CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)




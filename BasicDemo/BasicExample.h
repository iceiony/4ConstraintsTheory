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

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#define angleMin -1
#define angleMax 1
#define angleStep 0.2
#define yMin -0.5
#define yMax  1
#define yStep 0.1
#define zMin  -1
#define zMax  1
#define zStep  0.1
#define toolMass 1
#define objMass 1

class CommonExampleInterface*    BasicExampleCreateFunc(struct CommonExampleOptions& options);

struct BasicExample : public CommonRigidBodyBase
{
private:
    btVector3 toolDefaultLocation;
    btVector3 objDefaultLocation;
    btRigidBody *toolBody;
    btRigidBody *objBody;

    btScalar zOffset;
    btScalar yOffset;
    btScalar angle;
    bool finished;




    virtual void createGround();
    virtual btRigidBody * loadMeshObject(const char *fileName, const btVector3 &position, const btScalar &mass, const float scaleFactor);
public:
    BasicExample(struct GUIHelperInterface* helper)
            :CommonRigidBodyBase(helper)
    {
        toolDefaultLocation = btVector3(2.2,1.25,0);
        objDefaultLocation = btVector3(-2.2,1,0);
    }
    virtual ~BasicExample(){}
    virtual void initPhysics();
    virtual void renderScene();
    virtual void stepSimulation(float deltaTime);

    void resetCamera()
    {
        float dist = 15;
        float pitch = 32;
        float yaw = 35;
        float targetPos[3]={0,0.46,0};
        m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
    }


    void nextScenario();

    const btScalar &nextY();

    const btScalar &nextZ();

    btQuaternion nextRotation();

    bool isFinished();
};


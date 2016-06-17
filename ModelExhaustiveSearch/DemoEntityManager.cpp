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
#include "Util.h"
#include "DemoCamera.h"
#include "DebugDisplay.h"
#include "DemoCameraListener.h"


#define MAX_PHYSICS_FPS                120.0f


DemoEntityManager::ButtonKey::ButtonKey(bool state)
        : m_state(state), m_memory0(false), m_memory1(false) {
}


DemoEntityManager::DemoEntityManager() :
        dList<DemoEntity *>(), m_world(NULL), m_sky(NULL), m_microsecunds(0), m_currentListenerTimestep(0.0f),
        m_physicsUpdate(true), m_reEntrantUpdate(false), m_renderHoodContext(NULL), m_renderHood(NULL), m_font(0),
        m_fontImage(0), m_cameraManager(NULL) {
    // initialized the physics world for the new scene
    Cleanup();

    ResetTimer();

    dTimeTrackerSetThreadName ("mainThread");;
}


DemoEntityManager::~DemoEntityManager(void) {
    // is we are run asynchronous we need make sure no update in on flight.
    if (m_world) {
        NewtonWaitForUpdateToFinish(m_world);
    }

    Cleanup();

    // destroy the empty world
    if (m_world) {
        NewtonDestroy(m_world);
        m_world = NULL;
    }
    dAssert (NewtonGetMemoryUsed() == 0);
}

void DemoEntityManager::RemoveEntity (dListNode* const entNode)
{
    DemoEntity* const entity = entNode->GetInfo();
    entity->Release();
    Remove(entNode);
}

void DemoEntityManager::RemoveEntity (DemoEntity* const ent)
{
    for (dListNode* node = dList<DemoEntity*>::GetFirst(); node; node = node->GetNext()) {
        if (node->GetInfo() == ent) {
            RemoveEntity (node);
            break;
        }
    }
}

void DemoEntityManager::Cleanup() {
    // is we are run asynchronous we need make sure no update in on flight.
    if (m_world) {
        NewtonWaitForUpdateToFinish(m_world);
    }

    // destroy all remaining visual objects
    while (dList<DemoEntity *>::GetFirst()) {
        RemoveEntity(dList<DemoEntity *>::GetFirst());
    }

    m_sky = NULL;

    // destroy the Newton world
    if (m_world) {
        // get serialization call back before destroying the world
        NewtonDestroy(m_world);
        m_world = NULL;
    }

    //	memset (&demo, 0, sizeof (demo));
    // check that there are no memory leak on exit
    dAssert (NewtonGetMemoryUsed() == 0);

    // create the newton world
    m_world = NewtonCreate();

    // link the work with this user data
    NewtonWorldSetUserData(m_world, this);

    // set joint serialization call back
    CustomJoint::Initalize(m_world);

    // add all physics pre and post listeners
    //	m_preListenerManager.Append(new DemoVisualDebugerListener("visualDebuger", m_world));
    //	m_postListenerManager.Append (new DemoAIListener("aiManager"));
//	new DemoEntityListener (this);
    m_cameraManager = new DemoCameraListener(this);
    // set the default parameters for the newton world
    // set the simplified solver mode (faster but less accurate)
    NewtonSetSolverModel(m_world, 4);

    // newton 300 does not have world size, this is better controlled by the client application
    //dVector minSize (-500.0f, -500.0f, -500.0f);
    //dVector maxSize ( 500.0f,  500.0f,  500.0f);
    //NewtonSetWorldSize (m_world, &minSize[0], &maxSize[0]);

    // set the performance track function
    //NewtonSetPerformanceClock (m_world, dRuntimeProfiler::GetTimeInMicrosenconds);

    // clean up all caches the engine have saved
    NewtonInvalidateCache(m_world);

    // Set the Newton world user data
    NewtonWorldSetUserData(m_world, this);


    // we start without 2d render
    m_renderHood = NULL;
    m_renderHoodContext = NULL;
}

void DemoEntityManager::ResetTimer() {
    dResetTimer();
    m_microsecunds = dGetTimeInMicrosenconds();
}

void DemoEntityManager::PushTransparentMesh(const DemoMeshInterface *const mesh) {
    dMatrix matrix;
    glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);
    TransparentMesh entry(matrix, (DemoMesh *) mesh);
    m_tranparentHeap.Push(entry, matrix.m_posit.m_z);
}


DemoCamera* DemoEntityManager::GetCamera() const
{
    return m_cameraManager->GetCamera();
}

void DemoEntityManager::SetCameraMouseLock (bool state)
{
    m_cameraManager->SetCameraMouseLock(state);
}

void DemoEntityManager::SetCameraMatrix (const dQuaternion& rotation, const dVector& position)
{
    m_cameraManager->SetCameraMatrix(this, rotation, position);
}

void DemoEntityManager::InitGraphicsSystem() {
    GLenum err = glewInit();

    // if Glew doesn't initialize correctly.

    if (err != GLEW_OK) {
        printf("Failed to init glew");
    }

}

dFloat DemoEntityManager::GetPhysicsTime() {
    return m_mainThreadPhysicsTime;
}

void DemoEntityManager::UpdatePhysics(dFloat timestep) {
    // update the physics
    if (m_world) {

        dFloat timestepInSecunds = 1.0f / MAX_PHYSICS_FPS;
        unsigned64 timestepMicrosecunds = unsigned64(timestepInSecunds * 1000000.0f);

        unsigned64 currentTime = dGetTimeInMicrosenconds();
        unsigned64 nextTime = currentTime - m_microsecunds;
        if (nextTime > timestepMicrosecunds * 2) {
            m_microsecunds = currentTime - timestepMicrosecunds * 2;
            nextTime = currentTime - m_microsecunds;
        }

        //while (nextTime >= timestepMicrosecunds)
        if (nextTime >= timestepMicrosecunds) {
            unsigned64 time0 = dGetTimeInMicrosenconds();

            dTimeTrackerEvent(__FUNCTION__);
            // run the newton update function
            if (!m_reEntrantUpdate) {
                m_reEntrantUpdate = true;
                if (m_physicsUpdate && m_world) {
//					ClearDebugDisplay(m_world);
                    NewtonUpdateAsync(m_world, timestepInSecunds);
                }
                m_reEntrantUpdate = false;
            }
            m_microsecunds += timestepMicrosecunds;

            unsigned64 time1 = dGetTimeInMicrosenconds();
            m_mainThreadPhysicsTime = dFloat((time1 - time0) / 1000000.0f);
        }
    }
}

dFloat DemoEntityManager::CalculateInterpolationParam() const {
    unsigned64 timeStep = dGetTimeInMicrosenconds() - m_microsecunds;
    dFloat param = (dFloat(timeStep) * MAX_PHYSICS_FPS) / 1.0e6f;
    dAssert (param >= 0.0f);
    if (param > 1.0f) {
        param = 1.0f;
    }
    return param;
}

void DemoEntityManager::RenderFrame() {
    dTimeTrackerEvent(__FUNCTION__);

    dFloat timestep = dGetElapsedSeconds();

    // update the the state of all bodies in the scene
    UpdatePhysics(timestep);

    // Get the interpolated location of each body in the scene
    m_cameraManager->InterpolateMatrices(this, CalculateInterpolationParam());

    // Our shading model--Goraud (smooth).
    glShadeModel(GL_SMOOTH);

    // Culling.
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);

    //	glEnable(GL_DITHER);

    // z buffer test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);

    glClearColor(0.5f, 0.5f, 0.5f, 0.0f);
    //glClear( GL_COLOR_BUFFER_BIT );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    // set default lightning
    //	glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);

    // make sure the model view matrix is set to identity before setting world space ligh sources
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    dFloat cubeColor[] = {1.0f, 1.0f, 1.0f, 1.0};
    glMaterialParam(GL_FRONT, GL_SPECULAR, cubeColor);
    glMaterialParam(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
    glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    // one light form the Camera eye point
    GLfloat lightDiffuse0[] = {0.5f, 0.5f, 0.5f, 0.0};
    GLfloat lightAmbient0[] = {0.0f, 0.0f, 0.0f, 0.0};
    dVector camPosition(m_cameraManager->GetCamera()->m_matrix.m_posit);
    GLfloat lightPosition0[] = {camPosition.m_x, camPosition.m_y, camPosition.m_z};

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightDiffuse0);
    glEnable(GL_LIGHT0);


    // set just one directional light
    GLfloat lightDiffuse1[] = {0.7f, 0.7f, 0.7f, 0.0};
    GLfloat lightAmbient1[] = {0.2f, 0.2f, 0.2f, 0.0};
    GLfloat lightPosition1[] = {-500.0f, 200.0f, 500.0f, 0.0};

    glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, lightDiffuse1);
    glEnable(GL_LIGHT1);

    // update Camera
    m_cameraManager->GetCamera()->SetViewMatrix(GetWidth(), GetHeight());

    // render all entities
    for (dListNode *node = dList<DemoEntity *>::GetFirst(); node; node = node->GetNext()) {
        DemoEntity *const entity = node->GetInfo();
        glPushMatrix();
        entity->Render(timestep, this);
        glPopMatrix();
    }

    if (m_tranparentHeap.GetCount()) {
        dMatrix modelView;
        glGetFloat (GL_MODELVIEW_MATRIX, &modelView[0][0]);
        while (m_tranparentHeap.GetCount()) {
            const TransparentMesh &transparentMesh = m_tranparentHeap[0];
            glLoadIdentity();
            glLoadMatrix(&transparentMesh.m_matrix[0][0]);
            transparentMesh.m_mesh->RenderTransparency();
            m_tranparentHeap.Pop();
        }
        glLoadMatrix(&modelView[0][0]);
    }


    m_cameraManager->RenderPickedTarget();

    RenderContactPoints(GetNewton());

    RenderCenterOfMass(GetNewton());

    DebugRenderWorldCollision(GetNewton(), m_lines);

    int lineNumber = 130 + 22;

    if (m_renderHood) {

        // set display for 2d render mode

        dFloat width = GetWidth();
        dFloat height = GetHeight();

        glColor3f(1.0, 1.0, 1.0);

        glPushMatrix();
        glMatrixMode(GL_PROJECTION);

        glLoadIdentity();
        gluOrtho2D(0, width, 0, height);

        glPushMatrix();
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // render 2d display
        m_renderHood(this, m_renderHoodContext, lineNumber);

        // restore display mode
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();


        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }


    // draw everything and swap the display buffer
    glFlush();
}

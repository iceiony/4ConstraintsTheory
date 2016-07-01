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
#include "Camera.h"
#include "DebugDisplay.h"
#include "CameraListener.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

GraphicsManager::GraphicsManager(NewtonWorld *world) :
        dList<GraphicsEntity *>(), m_world(world),
        m_physicsPaused(false) ,m_cameraManager(new CameraListener(this)) {

    m_instance = this;
    m_window = InitialiseGraphics();

    NewtonWorldSetUserData(m_world, this);

    glfwSetKeyCallback(m_window,PauseKeyCallback);
}


GraphicsManager::~GraphicsManager(void) {
    glfwTerminate();

    // destroy all remaining visual objects
    while (dList<GraphicsEntity *>::GetFirst()) {
        RemoveEntity(dList<GraphicsEntity *>::GetFirst());
    }
}

void GraphicsManager::RemoveEntity(dListNode *const entNode) {
    GraphicsEntity *const entity = entNode->GetInfo();
    entity->Release();
    Remove(entNode);
}

void GraphicsManager::PushTransparentMesh(const GraphicsMeshInterface *const mesh) {
    dMatrix matrix;
    glGetFloat (GL_MODELVIEW_MATRIX, &matrix[0][0]);
    TransparentMesh entry(matrix, (GraphicsMesh *) mesh);
    m_tranparentHeap.Push(entry, matrix.m_posit.m_z);
}


void GraphicsManager::SetCameraMatrix(const dQuaternion &rotation, const dVector &position) {
    m_cameraManager->SetCameraMatrix(this, rotation, position);
}

dFloat GraphicsManager::CalculateInterpolationParam(dFloat timeStep) const {
    dFloat param = (timeStep * MAX_PHYSICS_FPS) / 1.0e6f;
    dAssert (param >= 0.0f);
    if (param > 1.0f) {
        param = 1.0f;
    }
    return param;
}

void GraphicsManager::UpdateGraphics(dFloat timeStep) {
    if (IsPhysicsPaused()){
        m_cameraManager->PausedStateUpdateCamera(this);
    }

    RenderFrame(timeStep);

    glfwSwapBuffers(m_window);

    glfwPollEvents();
}

void GraphicsManager::RenderFrame(dFloat timeStep) {
    dTimeTrackerEvent(__FUNCTION__);

    // Get the interpolated location of each body in the scene
    m_cameraManager->InterpolateMatrices(this, CalculateInterpolationParam(timeStep));

    glClear(GL_COLOR_BUFFER_BIT);

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

    glClearColor(0.86f, 0.96f, 1.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set default lightning
    glDisable(GL_BLEND);
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
    GLfloat lightDiffuse0[] = {0.1f, 0.1f, 0.1f, 0.0};
    GLfloat lightAmbient0[] = {0.0f, 0.0f, 0.0f, 0.0};
    dVector camPosition(m_cameraManager->GetCamera()->m_matrix.m_posit);
    GLfloat lightPosition0[] = {camPosition.m_x, camPosition.m_y, camPosition.m_z};

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightDiffuse0);
    glEnable(GL_LIGHT0);


    // set just one directional light
    GLfloat lightDiffuse1[] = {0.2f, 0.2f, 0.2f, 0.0};
    GLfloat lightAmbient1[] = {0.7f, 0.7f, 0.7f, 0.0};
    GLfloat lightPosition1[] = {.0f, 100.0f, .0f, 0.0};

    glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, lightDiffuse1);
    glEnable(GL_LIGHT1);

    // update Camera
    m_cameraManager->GetCamera()->SetViewMatrix(GetWidth(), GetHeight());

    // render all entities
    for (dListNode *node = dList<GraphicsEntity *>::GetFirst(); node; node = node->GetNext()) {
        GraphicsEntity *const entity = node->GetInfo();
        glPushMatrix();
        entity->Render(timeStep, this);
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


    RenderContactPoints(m_world);

    RenderCenterOfMass(m_world);

    DebugRenderWorldCollision(m_world, m_lines);

    RenderAABB(m_world);

    // draw everything and swap the display buffer
    glFlush();
}

void GraphicsManager::SetWindowSize(int width, int height) {
    this->m_width = width;
    this->m_height = height;

}

GLFWwindow *const GraphicsManager::GetRootWindow() const {
    return m_window;
}

GLFWwindow * GraphicsManager::InitialiseGraphics() {
    if (!glfwInit()) {
        throw "Could not init GLFW \n";

    };

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Simulation", NULL, NULL);

    if (!window) {
        throw "Could not open OpenGL m_window \n";
    }

    /* Make the m_window's context current */
    glfwMakeContextCurrent(window);

    SetWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glfwSetWindowSizeCallback(window, WindowResizeCallback);

    return window;
}

GraphicsManager *GraphicsManager::m_instance;

void GraphicsManager::WindowResizeCallback(GLFWwindow *window, int width, int height) {
    m_instance->SetWindowSize(width, height);
}

void GraphicsManager::PauseKeyCallback(GLFWwindow *window,int key,int scancode,int action,int mod){
    if (key == GLFW_KEY_P && action == GLFW_PRESS)
        m_instance->TogglePause();
}

void GraphicsManager::TogglePause() {
    this->m_physicsPaused = !this->m_physicsPaused;
}

bool GraphicsManager::IsPhysicsPaused() {
    return this->m_physicsPaused;
}


bool GraphicsManager::IsWindowClosed() {
    return glfwWindowShouldClose(m_window) == 1;
}

void GraphicsManager::Register(NewtonBody * body,dVector color = GRAY) {
    NewtonCollision * collision = NewtonBodyGetCollision(body);
    NewtonMesh * mesh = NewtonMeshCreateFromCollision(collision);

    dMatrix position;
    NewtonBodyGetMatrix(body,&position[0][0]);

    //Visual mesh rendering was taken from Newton demo code
    GraphicsMesh* const visualMesh = new GraphicsMesh(mesh,color);
    GraphicsEntity *const entity = new GraphicsEntity(position, NULL);
    Append(entity);
    if (mesh) {
        entity->SetMesh(visualMesh, dGetIdentityMatrix());
    }

    NewtonBodySetUserData(body, entity);

    visualMesh->Release();

    // set the transform call back function
    NewtonBodySetTransformCallback(body, GraphicsEntity::TransformCallback);
}

/**
 * Sets the camera with given up-down, left-right angles in degrees
 */
void GraphicsManager::SetCamera(dVector origin, dFloat upAngle = 0.0f, dFloat leftAngle = 0.0f ) {
    dMatrix upRotation = dRollMatrix( upAngle * 3.1416f /180.0f);
    dMatrix leftRotation = dYawMatrix( leftAngle * 3.1416f /180.0f);

    SetCameraMatrix( dQuaternion(upRotation * leftRotation), origin );
}











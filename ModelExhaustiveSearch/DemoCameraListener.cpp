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



// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include "DemoCamera.h"
#include "DemoCameraListener.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


#define D_CAMERA_LISTENER_NAMNE "cameraListener"

DemoCameraListener::DemoCameraListener(DemoEntityManager* const scene)
	:DemoListenerBase(scene, D_CAMERA_LISTENER_NAMNE)
	,m_camera (new DemoCamera())
	,m_mousePosX(0)
	,m_mousePosY(0)
	,m_yaw (m_camera->GetYawAngle())
	,m_pitch (m_camera->GetPichAngle())
	,m_yawRate (0.03f)
	,m_pitchRate (0.03f)
	,m_frontSpeed(15.0f)
	,m_sidewaysSpeed(10.0f)
	,m_pickedBodyParam(0.0f)
	,m_prevMouseState(false)
	,m_mouseLockState(false)
	,m_pickedBodyTargetPosition(0.0f)
	,m_pickedBodyLocalAtachmentPoint(0.0f)
	,m_pickedBodyLocalAtachmentNormal(0.0f)
	,m_targetPicked(NULL)
	,m_bodyDestructor(NULL)
{
}

DemoCameraListener::~DemoCameraListener()
{
	m_camera->Release();
}

void DemoCameraListener::PreUpdate (const NewtonWorld* const world, dFloat timestep)
{
	// update the camera;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	GLFWwindow * const mainWin = scene->GetRootWindow();

	dMatrix targetMatrix (m_camera->GetNextMatrix());

	double mouseX,mouseY;
	glfwGetCursorPos(mainWin, &mouseX, &mouseY);

	// slow down the Camera if we have a Body
	int state = glfwGetKey(mainWin, GLFW_KEY_LEFT_SHIFT );
	dFloat slowDownFactor = state == GLFW_PRESS ? 0.5f/2.0f : 0.5f;

	// do camera translation
	if (GetKeyState (mainWin,'W')) {
		targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (GetKeyState (mainWin,'S')) {
		targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
	}
	if (GetKeyState (mainWin,'A')) {
		targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}
	if (GetKeyState (mainWin,'D')) {
		targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (GetKeyState (mainWin,'Q')) {
		targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	if (GetKeyState (mainWin,'E')) {
		targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
	}

	// do camera rotation, only if we do not have anything picked
	state = glfwGetMouseButton(mainWin, GLFW_MOUSE_BUTTON_LEFT);
	bool buttonState = m_mouseLockState || state == GLFW_PRESS;
	if (!m_targetPicked && buttonState) {
        int mouseSpeedX = mouseX - m_mousePosX;
        int mouseSpeedY = mouseY - m_mousePosY;

        if (mouseSpeedX > 0) {
            m_yaw = dMod(m_yaw + m_yawRate, 2.0f * 3.1416f);
        } else if (mouseSpeedX < 0){
            m_yaw = dMod(m_yaw - m_yawRate, 2.0f * 3.1416f);
        }

        if (mouseSpeedY > 0) {
            m_pitch += m_pitchRate;
        } else if (mouseSpeedY < 0){
            m_pitch -= m_pitchRate;
        }
        m_pitch = dClamp(m_pitch, dFloat (-80.0f * 3.1416f / 180.0f), dFloat (80.0f * 3.1416f / 180.0f));
    }

	m_mousePosX = mouseX;
	m_mousePosY = mouseY;

	dMatrix matrix (dRollMatrix(m_pitch) * dYawMatrix(m_yaw));
	dQuaternion rot (matrix);
	m_camera->SetMatrix (*scene, rot, targetMatrix.m_posit);
}

void DemoCameraListener::PostUpdate (const NewtonWorld* const world, dFloat timestep)
{
}

void DemoCameraListener::SetCameraMouseLock (bool loockState)
{
	m_mouseLockState = loockState;
}

void DemoCameraListener::InterpolateMatrices (DemoEntityManager* const scene, dFloat param)
{
	NewtonWorld* const world = scene->GetNewton();

	// interpolate the Camera matrix;
	m_camera->InterpolateMatrix (*scene, param);

	// interpolate the location of all entities in the world
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
		if (entity) {
			entity->InterpolateMatrix (*scene, param);
		}
	}
}

void DemoCameraListener::OnBodyDestroy (NewtonBody* const body)
{
	// remove the references pointer because the body is going to be destroyed
	m_targetPicked = NULL;
	m_bodyDestructor = NULL;
}

bool DemoCameraListener::GetKeyState(GLFWwindow *const window, char key) {
	int state = glfwGetKey(window, key);
	return state == GLFW_PRESS;
}





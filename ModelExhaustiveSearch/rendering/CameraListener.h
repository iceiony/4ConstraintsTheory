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


// RenderPrimitive.h: interface for the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __DEMO_CAMERA_LISTENER_H__
#define __DEMO_CAMERA_LISTENER_H__

#include "Util.h"
#include "GraphicsEntity.h"

class CameraListener: public ListenerBase
{
	public:
	CameraListener(GraphicsManager* const scene);
	~CameraListener();

	Camera* GetCamera() const
	{
		return m_camera;
	}

	void SetCameraMatrix (GraphicsManager* const scene, const dQuaternion& rotation, const dVector& position)
	{
		m_camera->SetMatrix(*scene, rotation, position);
		m_camera->SetMatrix(*scene, rotation, position);
		m_yaw = m_camera->GetYawAngle();
		m_pitch = m_camera->GetPichAngle();
	}

	void InterpolateMatrices (GraphicsManager* const scene, dFloat timeStepFraction);

	private:
	virtual void PreUpdate (const NewtonWorld* const world, dFloat timestep);
	virtual void PostUpdate (const NewtonWorld* const world, dFloat timestep);

	virtual void OnBodyDestroy (NewtonBody* const body);

	Camera* m_camera;
	int m_mousePosX;
	int m_mousePosY;
	dFloat m_yaw;
	dFloat m_pitch;
	dFloat m_yawRate;
	dFloat m_pitchRate;
	dFloat m_frontSpeed;
	dFloat m_sidewaysSpeed;
	dFloat m_pickedBodyParam;

	bool m_prevMouseState;	
	bool m_mouseLockState;	
	dVector m_pickedBodyTargetPosition;
	dVector m_pickedBodyLocalAtachmentPoint;
	dVector m_pickedBodyLocalAtachmentNormal;
	NewtonBody* m_targetPicked;
	NewtonBodyDestructor m_bodyDestructor;
	friend class Camera;

	bool GetKeyState(GLFWwindow *const window, char key);
};

#endif 


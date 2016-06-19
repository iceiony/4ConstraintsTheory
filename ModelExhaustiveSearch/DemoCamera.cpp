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

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MOUSE_PICK_DAMP			 10.0f
#define MOUSE_PICK_STIFFNESS	 100.0f


DemoCamera::DemoCamera()
	:DemoEntity (dGetIdentityMatrix(), NULL) 
	,m_fov (60.0f * 3.1416f / 180.0f)
	,m_backPlane(2000.0f)
	,m_frontPlane (0.01f)
	,m_cameraYaw(0.0f)
	,m_cameraPitch(0.0f)
{
}

	
DemoCamera::~DemoCamera()
{
}

void DemoCamera::Render(dFloat timeStep, DemoEntityManager* const scene) const
{
}

dFloat DemoCamera::GetYawAngle() const
{
	return m_cameraYaw;
}

dFloat DemoCamera::GetPichAngle() const
{
	return m_cameraPitch;
}

void DemoCamera::SetViewMatrix(int width, int height)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint) width, (GLint) height);

	// set the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//m_backPlane = 10000.0f;
	gluPerspective(m_fov * 180.0f / 3.1416f, GLfloat (width) /GLfloat(height), m_frontPlane, m_backPlane);

	// set the model view matrix 
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	dVector pointOfInterest (m_matrix.m_posit + m_matrix.m_front);
	gluLookAt(m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z, 
		      pointOfInterest.m_x, pointOfInterest.m_y, pointOfInterest.m_z, 
			  m_matrix.m_up.m_x, m_matrix.m_up.m_y, m_matrix.m_up.m_z);	


	glGetIntegerv(GL_VIEWPORT, (GLint*)&m_viewport); 
	glGetDoublev(GL_MODELVIEW_MATRIX, m_modelViewMatrix); 
	glGetDoublev(GL_PROJECTION_MATRIX, m_projectionViewMatrix); 
}

void DemoCamera::SetMatrix (DemoEntityManager& scene, const dQuaternion& rotation, const dVector& position)
{
	dMatrix matrix (rotation, position);
	m_cameraPitch = dAsin (matrix.m_front.m_y);
	m_cameraYaw = dAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);

	DemoEntity::SetMatrix (scene, rotation, position);
}




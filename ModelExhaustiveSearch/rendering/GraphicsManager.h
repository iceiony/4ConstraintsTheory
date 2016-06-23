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

#ifndef __DEMO_ENTITY_MANAGER_H__
#define __DEMO_ENTITY_MANAGER_H__

#include "GraphicsMesh.h"
#include "ListenerBase.h"
#include "dHighResolutionTimer.h"
#include <GLFW/glfw3.h>

#define RED   dVector(1.,.3,.3)
#define GREEN dVector(.3,.95,.3)
#define BLUE  dVector(.5,.5,1.)
#define GRAY  dVector(.8,.8,.8)

//class GraphicsMesh;
class GraphicsEntity;
class Camera;
class NewtonDemos;
class GraphicsManager;
class CameraListener;


typedef void (*RenderHoodCallback) (GraphicsManager* const manager, void* const context, int lineNumber);

class GraphicsManager: public dList <GraphicsEntity*>
{
	public:
    class TransparentMesh
    {
        public: 
        TransparentMesh()
            :m_matrix(dGetIdentityMatrix())
            ,m_mesh(NULL)
        {
        }

        TransparentMesh(const dMatrix& matrix, const GraphicsMesh* const mesh)
            :m_matrix(matrix)
            ,m_mesh(mesh)
        {
        }

        dMatrix m_matrix;
        const GraphicsMesh* m_mesh;
    };

    class TransparentHeap: public dUpHeap <TransparentMesh, dFloat>
    {
        public:
        TransparentHeap()
            :dUpHeap <TransparentMesh, dFloat>(256)
        {
        }
    };

	class EntityDictionary: public dTree<GraphicsEntity*, dScene::dTreeNode*>
	{
	};

	GraphicsManager(NewtonWorld * world);
	~GraphicsManager(void);

	void RenderFrame(dFloat timeStep);

	int GetWidth() const;
	int GetHeight() const;

	NewtonWorld* GetNewton() const;

	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);

    void PushTransparentMesh (const GraphicsMeshInterface* const mesh);

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	void RemoveEntity (dList<GraphicsEntity*>::dListNode* const entNode);
	static void WindowResizeCallback(GLFWwindow *window, int width, int height);

	void SetWindowSize(int width, int height);

	GLFWwindow *const GetRootWindow() const;

	bool IsWindowClosed();

	void UpdateGraphics(unsigned64 i);

	void Register(NewtonBody *body,dVector color);

	void SetCamera(dVector origin, dFloat leftAngle , dFloat upAngle );

private:


	dFloat CalculateInterpolationParam(dFloat simulationTime) const;

	NewtonWorld* m_world;

	static GraphicsManager *instance;
	CameraListener* m_cameraManager;

	GLFWwindow *window;

    TransparentHeap m_tranparentHeap;

	//screen width and height required for rendring
	int width;
	int height;

	void InitialiseGraphics();
};

// for simplicity we are not going to run the demo in a separate thread at this time
// this confuses many user int thinking it is more complex than it really is  
inline void GraphicsManager::Lock(unsigned& atomicLock)
{
	while (NewtonAtomicSwap((int*)&atomicLock, 1)) {
		NewtonYield();
	}
}

inline void GraphicsManager::Unlock(unsigned& atomicLock)
{
	NewtonAtomicSwap((int*)&atomicLock, 0);
}


inline NewtonWorld* GraphicsManager::GetNewton() const
{
	return m_world;
}



inline int GraphicsManager::GetWidth() const
{ 
	return this->width;
}

inline int GraphicsManager::GetHeight() const
{ 
	return this->height;
}

#endif

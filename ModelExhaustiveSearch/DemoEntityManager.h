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

#include "DemoMesh.h"
#include "DemoListenerBase.h"
#include "dHighResolutionTimer.h"
#include <GLFW/glfw3.h>

//class DemoMesh;
class DemoEntity;
class DemoCamera;
class NewtonDemos;
class DemoEntityManager;
class DemoCameraListener;


typedef void (*RenderHoodCallback) (DemoEntityManager* const manager, void* const context, int lineNumber);

class DemoEntityManager: public dList <DemoEntity*>
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

        TransparentMesh(const dMatrix& matrix, const DemoMesh* const mesh)
            :m_matrix(matrix)
            ,m_mesh(mesh)
        {
        }

        dMatrix m_matrix;
        const DemoMesh* m_mesh;
    };

    class TransparentHeap: public dUpHeap <TransparentMesh, dFloat>
    {
        public:
        TransparentHeap()
            :dUpHeap <TransparentMesh, dFloat>(256)
        {
        }
    };

	class EntityDictionary: public dTree<DemoEntity*, dScene::dTreeNode*>
	{
	};

	DemoEntityManager(GLFWwindow * window);
	~DemoEntityManager(void);

	void ResetTimer();
	void RenderFrame ();
	void UpdatePhysics();

	int GetWidth() const;
	int GetHeight() const;

	NewtonWorld* GetNewton() const;

	void SetCameraMatrix (const dQuaternion& rotation, const dVector& position);

    void PushTransparentMesh (const DemoMeshInterface* const mesh); 

	void Lock(unsigned& atomicLock);
	void Unlock(unsigned& atomicLock);

	void Cleanup ();
	void RemoveEntity (dList<DemoEntity*>::dListNode* const entNode);

	void SetWindowSize(int width, int height);

	GLFWwindow *const GetRootWindow() const;

private:

	dFloat CalculateInterpolationParam() const;

	NewtonWorld* m_world;

	DemoEntity* m_sky;
	unsigned64 m_microseconds;
	dFloat m_currentListenerTimestep;
	bool m_physicsUpdate;
	bool m_reEntrantUpdate;
	void* m_renderHoodContext;
	RenderHoodCallback m_renderHood;
	GLuint m_font;
	GLuint m_fontImage;
	DemoCameraListener* m_cameraManager;

	GLFWwindow *window;

    TransparentHeap m_tranparentHeap;

	friend class NewtonDemos;
	//friend class dRuntimeProfiler;
	friend class DemoEntityListener;
	friend class DemoListenerManager;

	//screen width and height required for rendring
	int width;
	int height;
};

// for simplicity we are not going to run the demo in a separate thread at this time
// this confuses many user int thinking it is more complex than it really is  
inline void DemoEntityManager::Lock(unsigned& atomicLock)
{
	while (NewtonAtomicSwap((int*)&atomicLock, 1)) {
		NewtonYield();
	}
}

inline void DemoEntityManager::Unlock(unsigned& atomicLock)
{
	NewtonAtomicSwap((int*)&atomicLock, 0);
}


inline NewtonWorld* DemoEntityManager::GetNewton() const
{
	return m_world;
}



inline int DemoEntityManager::GetWidth() const 
{ 
	return this->width;
}

inline int DemoEntityManager::GetHeight() const 
{ 
	return this->height;
}

#endif

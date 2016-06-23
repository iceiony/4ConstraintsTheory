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

#ifndef __DEMO_ENTITY_H__
#define __DEMO_ENTITY_H__

#include "GraphicsManager.h"
class DemoMeshInterface;


class DemoEntity: public dHierarchy<DemoEntity>, virtual public dClassInfo
{
	public:

	class UserData
	{
		public:
		UserData()
		{
		}

		virtual ~UserData()
		{
		}

		virtual void OnRender (dFloat timestep) const = 0;
		virtual void OnInterpolateMatrix (DemoEntityManager& world, dFloat param) const = 0;
	};

	DemoEntity(const DemoEntity& copyFrom);
	DemoEntity(const dMatrix& matrix, DemoEntity* const parent);
	DemoEntity(DemoEntityManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<DemoMeshInterface*, dScene::dTreeNode*>& meshCache, DemoEntityManager::EntityDictionary& entityDictionary, DemoEntity* const parent = NULL);
	virtual ~DemoEntity(void);

	void SetMesh (DemoMeshInterface* const m_mesh, const dMatrix& meshMatrix);

	UserData* GetUserData ();
	void SetUserData (UserData* const data);

	dBaseHierarchy* CreateClone () const;

	dMatrix CalculateGlobalMatrix (const DemoEntity* const root = NULL) const;

	dMatrix GetNextMatrix () const;
	dMatrix GetCurrentMatrix () const;
	virtual void SetMatrix(DemoEntityManager& world, const dQuaternion& rotation, const dVector& position);

	virtual void ResetMatrix(DemoEntityManager& world, const dMatrix& matrix);
	virtual void InterpolateMatrix (DemoEntityManager& world, dFloat param);

	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const;

	static void TransformCallback(const NewtonBody* const body, const dFloat* const matrix, int threadIndex);

	protected:



	mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step  

	dMatrix m_meshMatrix;
	DemoMeshInterface* m_mesh;
	UserData* m_userData;

	unsigned m_lock;
	dAddRtti(dClassInfo,);

	friend class DemoEntityManager;
};

#endif
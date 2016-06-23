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
class GraphicsMeshInterface;


class GraphicsEntity: public dHierarchy<GraphicsEntity>, virtual public dClassInfo
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
		virtual void OnInterpolateMatrix (GraphicsManager& world, dFloat param) const = 0;
	};

	GraphicsEntity(const GraphicsEntity& copyFrom);
	GraphicsEntity(const dMatrix& matrix, GraphicsEntity* const parent);
	GraphicsEntity(GraphicsManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<GraphicsMeshInterface*, dScene::dTreeNode*>& meshCache, GraphicsManager::EntityDictionary& entityDictionary, GraphicsEntity* const parent = NULL);
	virtual ~GraphicsEntity(void);

	void SetMesh (GraphicsMeshInterface* const m_mesh, const dMatrix& meshMatrix);

	UserData* GetUserData ();
	void SetUserData (UserData* const data);

	dBaseHierarchy* CreateClone () const;

	dMatrix CalculateGlobalMatrix (const GraphicsEntity* const root = NULL) const;

	dMatrix GetNextMatrix () const;
	dMatrix GetCurrentMatrix () const;
	virtual void SetMatrix(GraphicsManager& world, const dQuaternion& rotation, const dVector& position);

	virtual void ResetMatrix(GraphicsManager& world, const dMatrix& matrix);
	virtual void InterpolateMatrix (GraphicsManager& world, dFloat param);

	virtual void Render(dFloat timeStep, GraphicsManager* const scene) const;

	static void TransformCallback(const NewtonBody* const body, const dFloat* const matrix, int threadIndex);

	protected:



	mutable dMatrix m_matrix;			// interpolated matrix
	dVector m_curPosition;				// position one physics simulation step in the future
	dVector m_nextPosition;             // position at the current physics simulation step
	dQuaternion m_curRotation;          // rotation one physics simulation step in the future  
	dQuaternion m_nextRotation;         // rotation at the current physics simulation step  

	dMatrix m_meshMatrix;
	GraphicsMeshInterface* m_mesh;
	UserData* m_userData;

	unsigned m_lock;
	dAddRtti(dClassInfo,);

	friend class GraphicsManager;
};

#endif
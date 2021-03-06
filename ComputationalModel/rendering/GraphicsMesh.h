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


#ifndef _D_MESH_H_
#define _D_MESH_H_

#include "Util.h"

class GraphicsMesh;
class GraphicsManager;


class GraphicsMeshInterface: public dClassInfo
{
	public:
	GraphicsMeshInterface();
	~GraphicsMeshInterface();

	virtual void RenderTransparency () const = 0;
	virtual void Render (GraphicsManager* const scene) = 0;

	dAddRtti(dClassInfo,DOMMY_API);

	dString m_name;
	bool m_isVisible;
};

class GraphicsSubMesh
{
	public:
	GraphicsSubMesh ();
	~GraphicsSubMesh ();

	void Render() const;
	void AllocIndexData (int indexCount);

	void SetOpacity(dFloat opacity);

	int m_indexCount;
	unsigned *m_indexes;

	dFloat m_shiness;
	dVector m_ambient;
	dVector m_diffuse;
	dVector m_specular;
	dFloat m_opacity;
};


class GraphicsMesh: public GraphicsMeshInterface, public dList<GraphicsSubMesh>
{
	public:
	GraphicsMesh(const GraphicsMesh& mesh);
	GraphicsMesh(const char* const name);
	GraphicsMesh(NewtonMesh *const mesh, dVector color);
	GraphicsMesh(const dScene* const scene, dScene::dTreeNode* const meshNode);
	GraphicsMesh(const char* const name, const NewtonCollision* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat opacity = 1.0f);
	GraphicsMesh(const char* const name, dFloat* const elevation, int size, dFloat cellSize, dFloat texelsDensity, int tileSize);

	using dClassInfo::operator new;
	using dClassInfo::operator delete;

	GraphicsSubMesh* AddSubMesh();
	void AllocVertexData (int vertexCount);

    virtual void RenderTransparency () const;
	virtual void Render (GraphicsManager* const scene);

	void OptimizeForRender();

	protected:
	virtual ~GraphicsMesh();

	dAddRtti (GraphicsMeshInterface, DOMMY_API);
	
	void  ResetOptimization();
	void  SplitSegment(dListNode *const node, int maxIndexCount);


	public:
	int m_vertexCount;
	dFloat* m_uv;
	dFloat* m_vertex;
	dFloat* m_normal;
	unsigned m_optimizedOpaqueDiplayList;
	unsigned m_optimizedTransparentDiplayList;		
};

#endif 



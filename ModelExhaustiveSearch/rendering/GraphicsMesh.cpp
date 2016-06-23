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
#include "GraphicsMesh.h"
#include "GraphicsManager.h"


dInitRtti(GraphicsMeshInterface);
dInitRtti(GraphicsMesh);


#define USING_DISPLAY_LIST



#if defined(__APPLE__)
// NOTE: displaylists are horribly slow on OSX
// they cut the framerate in half
#	if defined(USING_DISPLAY_LIST)
#		undef USING_DISPLAY_LIST
#	endif
#endif

GraphicsMeshInterface::GraphicsMeshInterface()
	:dClassInfo()
	,m_name()
	,m_isVisible(true)
{
}

GraphicsMeshInterface::~GraphicsMeshInterface()
{
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
GraphicsSubMesh::GraphicsSubMesh ()
	:m_indexCount(0)
	,m_indexes(NULL)
	,m_shiness(80.0f)
	,m_ambient (0.8f, 0.8f, 0.8f, 1.0f)
	,m_diffuse (0.8f, 0.8f, 0.8f, 1.0f)
	,m_specular (1.0f, 1.0f, 1.0f, 1.0f)
	,m_opacity(1.0f)
{
}

GraphicsSubMesh::~GraphicsSubMesh ()
{
	if (m_indexes) {
		delete[] m_indexes;
	}
}

void GraphicsSubMesh::SetOpacity(dFloat opacity)
{
	m_opacity = opacity;
	m_ambient.m_w = opacity;
	m_diffuse.m_w = opacity;
	m_specular.m_w = opacity;
}

void GraphicsSubMesh::Render() const
{
	glMaterialParam(GL_FRONT, GL_SPECULAR, &m_specular.m_x);
	glMaterialParam(GL_FRONT, GL_AMBIENT, &m_ambient.m_x);
	glMaterialParam(GL_FRONT, GL_DIFFUSE, &m_diffuse.m_x);
	glMaterialf(GL_FRONT, GL_SHININESS, m_shiness);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glDisable(GL_TEXTURE_2D);

	glDrawElements (GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, m_indexes);
}

void GraphicsSubMesh::AllocIndexData (int indexCount)
{
	m_indexCount = indexCount;
	if (m_indexes) {
		delete[] m_indexes;
	}
	m_indexes = new unsigned [m_indexCount]; 
}



GraphicsMesh::GraphicsMesh(const char* const name)
	:GraphicsMeshInterface()
	,dList<GraphicsSubMesh>()
	,m_vertexCount(0)
	,m_uv (NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optimizedOpaqueDiplayList(0)	
	,m_optimizedTransparentDiplayList(0)
{
}

GraphicsMesh::GraphicsMesh(const dScene* const scene, dScene::dTreeNode* const meshNode)
	:GraphicsMeshInterface()
	,dList<GraphicsSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optimizedOpaqueDiplayList(0)
	,m_optimizedTransparentDiplayList(0)
{
	dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*)scene->GetInfoFromNode(meshNode);
	m_name = meshInfo->GetName();
	
	NewtonMesh* const mesh = meshInfo->GetMesh();

	// extract vertex data  from the newton mesh		
	AllocVertexData(NewtonMeshGetPointCount (mesh));
	NewtonMeshGetVertexStreams (mesh, 3 * sizeof (dFloat), (dFloat*) m_vertex,
									  3 * sizeof (dFloat), (dFloat*) m_normal,
									  2 * sizeof (dFloat), (dFloat*) m_uv, 
								      2 * sizeof (dFloat), (dFloat*) m_uv);

	// bake the matrix into the vertex array
	dMatrix matrix (meshInfo->GetPivotMatrix());
	matrix.TransformTriplex(m_vertex, 3 * sizeof (dFloat), m_vertex, 3 * sizeof (dFloat), m_vertexCount);
	matrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	matrix = (matrix.Inverse4x4()).Transpose();
	matrix.TransformTriplex(m_normal, 3 * sizeof (dFloat), m_normal, 3 * sizeof (dFloat), m_vertexCount);

	bool hasModifiers = false;
	dTree<dScene::dTreeNode*, dCRCTYPE> materialMap;
	for (void* ptr = scene->GetFirstChildLink(meshNode); ptr; ptr = scene->GetNextChildLink (meshNode, ptr)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(ptr);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*)info;
			dCRCTYPE id = material->GetId();
			materialMap.Insert(node, id);
		} else if (info->IsType(dGeometryNodeModifierInfo::GetRttiType())) {
			hasModifiers = true;
		}
	}

	// extract the materials index array for mesh
	void* const meshCookie = NewtonMeshBeginHandle (mesh); 
	for (int handle = NewtonMeshFirstMaterial (mesh, meshCookie); handle != -1; handle = NewtonMeshNextMaterial (mesh, meshCookie, handle)) {
		int materialIndex = NewtonMeshMaterialGetMaterial (mesh, meshCookie, handle); 
		int indexCount = NewtonMeshMaterialGetIndexCount (mesh, meshCookie, handle); 
		GraphicsSubMesh* const segment = AddSubMesh();

		dTree<dScene::dTreeNode*, dCRCTYPE>::dTreeNode* matNodeCache = materialMap.Find(materialIndex);
		if (matNodeCache) {
			dScene::dTreeNode* const matNode = matNodeCache->GetInfo();
			dMaterialNodeInfo* const material = (dMaterialNodeInfo*) scene->GetInfoFromNode(matNode);

			if (material->GetDiffuseTextId() != -1) {
				dScene::dTreeNode* const node = scene->FindTextureByTextId(matNode, material->GetDiffuseTextId());
				dAssert (node);
			}
			segment->m_shiness = material->GetShininess();
			segment->m_ambient = material->GetAmbientColor();
			segment->m_diffuse = material->GetDiffuseColor();
			segment->m_specular = material->GetSpecularColor();
			segment->SetOpacity(material->GetOpacity());
		}

		segment->AllocIndexData (indexCount);
		// for 16 bit indices meshes
		//NewtonMeshMaterialGetIndexStreamShort (mesh, meshCookie, handle, (short int*)segment->m_indexes); 

		// for 32 bit indices mesh
		NewtonMeshMaterialGetIndexStream (mesh, meshCookie, handle, (int*)segment->m_indexes); 
	}
	NewtonMeshEndHandle (mesh, meshCookie); 

	if (!hasModifiers) {
		// see if this mesh can be optimized
		OptimizeForRender ();
	}
}

GraphicsMesh::GraphicsMesh(NewtonMesh *const mesh, dVector color)
	:GraphicsMeshInterface()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optimizedOpaqueDiplayList(0)		
	,m_optimizedTransparentDiplayList(0)
{
	// extract vertex data  from the newton mesh		
	AllocVertexData(NewtonMeshGetPointCount (mesh));
	NewtonMeshGetVertexStreams (mesh, 3 * sizeof (dFloat), (dFloat*) m_vertex, 3 * sizeof (dFloat), (dFloat*) m_normal,	2 * sizeof (dFloat), (dFloat*) m_uv, 2 * sizeof (dFloat), (dFloat*) m_uv);

	// extract the materials index array for mesh
	void* const meshCookie = NewtonMeshBeginHandle (mesh); 
	for (int handle = NewtonMeshFirstMaterial (mesh, meshCookie); handle != -1; handle = NewtonMeshNextMaterial (mesh, meshCookie, handle)) {
		int indexCount = NewtonMeshMaterialGetIndexCount (mesh, meshCookie, handle);
		GraphicsSubMesh* const segment = AddSubMesh();

        segment->m_shiness = 0.0f;
        segment->m_ambient = color;
        segment->m_diffuse = color;
        segment->m_specular = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		segment->AllocIndexData (indexCount);
		// for 16 bit indices meshes
		//NewtonMeshMaterialGetIndexStreamShort (mesh, meshCookie, handle, (short int*)segment->m_indexes); 

		// for 32 bit indices mesh
		NewtonMeshMaterialGetIndexStream (mesh, meshCookie, handle, (int*)segment->m_indexes); 
	}
	NewtonMeshEndHandle (mesh, meshCookie); 

	// see if this mesh can be optimized
	OptimizeForRender ();
}

GraphicsMesh::GraphicsMesh(const GraphicsMesh& mesh)
	:GraphicsMeshInterface()
	,dList<GraphicsSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optimizedOpaqueDiplayList(0)		
	,m_optimizedTransparentDiplayList(0)
{
	AllocVertexData(mesh.m_vertexCount);
	memcpy (m_vertex, mesh.m_vertex, 3 * m_vertexCount * sizeof (dFloat));
	memcpy (m_normal, mesh.m_normal, 3 * m_vertexCount * sizeof (dFloat));
	memcpy (m_uv, mesh.m_uv, 2 * m_vertexCount * sizeof (dFloat));

	for (dListNode* nodes = mesh.GetFirst(); nodes; nodes = nodes->GetNext()) {
		GraphicsSubMesh* const segment = AddSubMesh();
		GraphicsSubMesh& srcSegment = nodes->GetInfo();

		segment->AllocIndexData (srcSegment.m_indexCount);
		memcpy (segment->m_indexes, srcSegment.m_indexes, srcSegment.m_indexCount * sizeof (unsigned));

		segment->m_shiness = srcSegment.m_shiness;
		segment->m_ambient = srcSegment.m_ambient;
		segment->m_diffuse = srcSegment.m_diffuse;
		segment->m_specular = srcSegment.m_specular;
	}

	// see if this mesh can be optimized
	OptimizeForRender ();
}

GraphicsMesh::GraphicsMesh(const char* const name, const NewtonCollision* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat opacity)
	:GraphicsMeshInterface()
	,dList<GraphicsSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optimizedOpaqueDiplayList(0)		
	,m_optimizedTransparentDiplayList(0)
{
	// create a helper mesh from the collision collision
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

	// apply the vertex normals
	NewtonMeshCalculateVertexNormals(mesh, 30.0f * 3.141592f/180.0f);

	// apply uv projections
	NewtonCollisionInfoRecord info;
	NewtonCollisionGetInfo (collision, &info);
	switch (info.m_collisionType) 
	{
		case SERIALIZE_ID_SPHERE:
		{
			NewtonMeshApplySphericalMapping(mesh,0);
			break;
		}

		case SERIALIZE_ID_CONE:
		case SERIALIZE_ID_CAPSULE:
		case SERIALIZE_ID_CYLINDER:
		case SERIALIZE_ID_CHAMFERCYLINDER:
		{
			NewtonMeshApplyCylindricalMapping(mesh, 0, 0 );
			break;
		}

		default:
		{
			break;
		}
	}


	// extract vertex data  from the newton mesh		
	int vertexCount = NewtonMeshGetPointCount (mesh); 
	AllocVertexData(vertexCount);
	NewtonMeshGetVertexStreams (mesh, 
								3 * sizeof (dFloat), (dFloat*) m_vertex,
								3 * sizeof (dFloat), (dFloat*) m_normal,
								2 * sizeof (dFloat), (dFloat*) m_uv, 
								2 * sizeof (dFloat), (dFloat*) m_uv);

	// extract the materials index array for mesh
	void* const geometryHandle = NewtonMeshBeginHandle (mesh); 
	for (int handle = NewtonMeshFirstMaterial (mesh, geometryHandle); handle != -1; handle = NewtonMeshNextMaterial (mesh, geometryHandle, handle)) {
		int indexCount = NewtonMeshMaterialGetIndexCount (mesh, geometryHandle, handle);

		GraphicsSubMesh* const segment = AddSubMesh();

		segment->SetOpacity(opacity);

		segment->AllocIndexData (indexCount);
		NewtonMeshMaterialGetIndexStream (mesh, geometryHandle, handle, (int*)segment->m_indexes); 
	}
	NewtonMeshEndHandle (mesh, geometryHandle); 

	// destroy helper mesh
	NewtonMeshDestroy(mesh);

	// optimize this mesh for hardware buffers if possible
	OptimizeForRender ();
}


GraphicsMesh::GraphicsMesh(const char* const name, dFloat* const elevation, int size, dFloat cellSize, dFloat texelsDensity, int tileSize)
	:GraphicsMeshInterface()
	,dList<GraphicsSubMesh>()
	,m_uv(NULL)
	,m_vertex(NULL)
	,m_normal(NULL)
	,m_optimizedOpaqueDiplayList(0)		
	,m_optimizedTransparentDiplayList(0)
{
	dFloat* elevationMap[4096];
	dVector* normalMap[4096];
	dFloat* const normalsPtr = new dFloat [size * size * 4];
	dVector* const normals = (dVector*)normalsPtr;

	for (int i = 0; i < size; i ++) {
		elevationMap[i] = &elevation[i * size];
		normalMap[i] = &normals[i * size];
	}

	memset (normals, 0, (size * size) * sizeof (dVector));
	for (int z = 0; z < size - 1; z ++) {
		for (int x = 0; x < size - 1; x ++) {
			dVector p0 ((x + 0) * cellSize, elevationMap[z + 0][x + 0], (z + 0) * cellSize);
			dVector p1 ((x + 1) * cellSize, elevationMap[z + 0][x + 1], (z + 0) * cellSize);
			dVector p2 ((x + 1) * cellSize, elevationMap[z + 1][x + 1], (z + 1) * cellSize);
			dVector p3 ((x + 0) * cellSize, elevationMap[z + 1][x + 0], (z + 1) * cellSize);

			dVector e10 (p1 - p0);
			dVector e20 (p2 - p0);
			dVector n0 (e20 * e10);
			n0 = n0.Scale ( 1.0f / dSqrt (n0 % n0));
			normalMap [z + 0][x + 0] += n0;
			normalMap [z + 0][x + 1] += n0;
			normalMap [z + 1][x + 1] += n0;

			dVector e30 (p3 - p0);
			dVector n1 (e30 * e20);
			n1 = n1.Scale ( 1.0f / dSqrt (n1 % n1));
			normalMap [z + 0][x + 0] += n1;
			normalMap [z + 1][x + 0] += n1;
			normalMap [z + 1][x + 1] += n1;
		}
	}

	for (int i = 0; i < size * size; i ++) {
		normals[i] = normals[i].Scale (1.0f / sqrtf (normals[i] % normals[i]));
	}
	
	AllocVertexData (size * size);

	dFloat* const vertex = m_vertex;
	dFloat* const normal = m_normal;
	dFloat* const uv = m_uv;

	int index0 = 0;
	for (int z = 0; z < size; z ++) {
		for (int x = 0; x < size; x ++) {
			vertex[index0 * 3 + 0] = x * cellSize;
			vertex[index0 * 3 + 1] = elevationMap[z][x];
			vertex[index0 * 3 + 2] = z * cellSize;

			normal[index0 * 3 + 0] = normalMap[z][x].m_x;
			normal[index0 * 3 + 1] = normalMap[z][x].m_y;
			normal[index0 * 3 + 2] = normalMap[z][x].m_z;

			uv[index0 * 2 + 0] = x * texelsDensity;
			uv[index0 * 2 + 1] = z * texelsDensity;
			index0 ++;
		}
	}

	int segmentsCount = (size - 1) / tileSize;
	for (int z0 = 0; z0 < segmentsCount; z0 ++) {
		int z = z0 * tileSize;
		for (int x0 = 0; x0 < segmentsCount; x0 ++ ) {
			int x = x0 * tileSize;

			GraphicsSubMesh* const tile = AddSubMesh();
			tile->AllocIndexData (tileSize * tileSize * 6);
			unsigned* const indexes = tile->m_indexes;

			int index1 = 0;
			int x1 = x + tileSize;
			int z1 = z + tileSize;
			for (int z2 = z; z2 < z1; z2 ++) {
				for (int x2 = x; x2 < x1; x2 ++) {
					int i0 = x2 + 0 + (z2 + 0) * size;
					int i1 = x2 + 1 + (z2 + 0) * size;
					int i2 = x2 + 1 + (z2 + 1) * size;
					int i3 = x2 + 0 + (z2 + 1) * size;

					indexes[index1 + 0] = i0;
					indexes[index1 + 1] = i2;
					indexes[index1 + 2] = i1;

					indexes[index1 + 3] = i0;
					indexes[index1 + 4] = i3;
					indexes[index1 + 5] = i2;
					index1 += 6;
				}
			}
		}
	}
	delete[] normalsPtr; 
	OptimizeForRender();
}



GraphicsMesh::~GraphicsMesh()
{
	if (m_vertex) {
		delete[] m_vertex;
		delete[] m_normal;
		delete[] m_uv;
		ResetOptimization();
	}
}

void GraphicsMesh::SplitSegment(dListNode *const node, int maxIndexCount)
{
	const GraphicsSubMesh& segment = node->GetInfo();
	if (segment.m_indexCount > maxIndexCount) {
		dVector minBox (1.0e10f, 1.0e10f, 1.0e10f, 0.0f);
		dVector maxBox (-1.0e10f, -1.0e10f, -1.0e10f, 0.0f);
		for (int i = 0; i < segment.m_indexCount; i ++) {
			int index = segment.m_indexes[i];
			for (int j = 0; j < 3; j ++) {
				minBox[j] = (m_vertex[index * 3 + j] < minBox[j]) ? m_vertex[index * 3 + j] : minBox[j];
				maxBox[j] = (m_vertex[index * 3 + j] > maxBox[j]) ? m_vertex[index * 3 + j] : maxBox[j];
			}
		}

		int index = 0;
		dFloat maxExtend = -1.0e10f;
		for (int j = 0; j < 3; j ++) {
			dFloat ext = maxBox[j] - minBox[j];
			if (ext > maxExtend ) {
				index = j;
				maxExtend = ext;
			}
		}

		int leftCount = 0;
		int rightCount = 0;
		dFloat spliteDist = (maxBox[index ] + minBox[index]) * 0.5f;
		for (int i = 0; i < segment.m_indexCount; i += 3) {
			bool isleft = true;
			for (int j = 0; j < 3; j ++) {
				int vertexIndex = segment.m_indexes[i + j];
				isleft &= (m_vertex[vertexIndex * 3 + index] < spliteDist);
			}
			if (isleft) {
				leftCount += 3;
			} else {
				rightCount += 3;
			}
		}
		dAssert (leftCount);
		dAssert (rightCount);

		dListNode* const leftNode = Append();
		dListNode* const rightNode = Append();
		GraphicsSubMesh* const leftSubMesh = &leftNode->GetInfo();
		GraphicsSubMesh* const rightSubMesh = &rightNode->GetInfo();
		leftSubMesh->AllocIndexData (leftCount);
		rightSubMesh->AllocIndexData (rightCount);

		leftCount = 0;
		rightCount = 0;
		for (int i = 0; i < segment.m_indexCount; i += 3) {
			bool isleft = true;
			for (int j = 0; j < 3; j ++) {
				int vertexIndex = segment.m_indexes[i + j];
				isleft &= (m_vertex[vertexIndex * 3 + index] < spliteDist);
			}
			if (isleft) {
				leftSubMesh->m_indexes[leftCount + 0] = segment.m_indexes[i + 0];
				leftSubMesh->m_indexes[leftCount + 1] = segment.m_indexes[i + 1];
				leftSubMesh->m_indexes[leftCount + 2] = segment.m_indexes[i + 2];
				leftCount += 3;
			} else {
				rightSubMesh->m_indexes[rightCount + 0] = segment.m_indexes[i + 0];
				rightSubMesh->m_indexes[rightCount + 1] = segment.m_indexes[i + 1];
				rightSubMesh->m_indexes[rightCount + 2] = segment.m_indexes[i + 2];
				rightCount += 3;
			}
		}

		SplitSegment(leftNode, maxIndexCount);
		SplitSegment(rightNode, maxIndexCount);
		Remove(node);
	}
}

void  GraphicsMesh::OptimizeForRender()
{
	// first make sure the previous optimization is removed
	ResetOptimization();

	dListNode* nextNode;
	for (dListNode* node = GetFirst(); node; node = nextNode) {
		GraphicsSubMesh& segment = node->GetInfo();
		nextNode = node->GetNext();
		if (segment.m_indexCount > 128 * 128 * 6) {
			SplitSegment(node, 128 * 128 * 6);
		}
	}
}

void  GraphicsMesh::ResetOptimization()
{
	if (m_optimizedOpaqueDiplayList) {
		glDeleteLists(m_optimizedOpaqueDiplayList, 1);
		m_optimizedOpaqueDiplayList = 0;
	}

	if (m_optimizedTransparentDiplayList) {
		glDeleteLists(m_optimizedTransparentDiplayList, 1);
		m_optimizedTransparentDiplayList = 0;
	}
}

void GraphicsMesh::AllocVertexData (int vertexCount)
{
	m_vertexCount = vertexCount;

	m_vertex = new dFloat[3 * m_vertexCount];
	m_normal = new dFloat[3 * m_vertexCount];
	m_uv = new dFloat[2 * m_vertexCount];
	memset (m_uv, 0, 2 * m_vertexCount * sizeof (dFloat));
}

GraphicsSubMesh* GraphicsMesh::AddSubMesh()
{
	return &Append()->GetInfo();
}


void GraphicsMesh::Render (GraphicsManager* const scene)
{
	if (m_isVisible) {
		if (m_optimizedTransparentDiplayList) {
			scene->PushTransparentMesh (this); 
		}

		if (m_optimizedOpaqueDiplayList) {
			glCallList(m_optimizedOpaqueDiplayList);
		} else if (!m_optimizedTransparentDiplayList) {
			glEnableClientState (GL_VERTEX_ARRAY);
			glEnableClientState (GL_NORMAL_ARRAY);
			glEnableClientState (GL_TEXTURE_COORD_ARRAY);

			glVertexPointer (3, GL_FLOAT, 0, m_vertex);
			glNormalPointer (GL_FLOAT, 0, m_normal);
			glTexCoordPointer (2, GL_FLOAT, 0, m_uv);

			for (dListNode* nodes = GetFirst(); nodes; nodes = nodes->GetNext()) {
				GraphicsSubMesh& segment = nodes->GetInfo();
				segment.Render();
			}
			glDisableClientState(GL_VERTEX_ARRAY);	// disable vertex arrays
			glDisableClientState(GL_NORMAL_ARRAY);	// disable normal arrays
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);	// disable normal arrays
		}
	}
}

void GraphicsMesh::RenderTransparency () const
{
	if (m_isVisible) {
		if (m_optimizedTransparentDiplayList) {
			glCallList(m_optimizedTransparentDiplayList);
		} else {
			glEnableClientState (GL_VERTEX_ARRAY);
			glEnableClientState (GL_NORMAL_ARRAY);
			glEnableClientState (GL_TEXTURE_COORD_ARRAY);

			glVertexPointer (3, GL_FLOAT, 0, m_vertex);
			glNormalPointer (GL_FLOAT, 0, m_normal);
			glTexCoordPointer (2, GL_FLOAT, 0, m_uv);

			for (dListNode* nodes = GetFirst(); nodes; nodes = nodes->GetNext()) {
				GraphicsSubMesh& segment = nodes->GetInfo();
				segment.Render();
			}
			glDisableClientState(GL_VERTEX_ARRAY);	// disable vertex arrays
			glDisableClientState(GL_NORMAL_ARRAY);	// disable normal arrays
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);	// disable normal arrays
		}
	}
}



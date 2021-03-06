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
#include "GraphicsEntity.h"

dInitRtti(GraphicsEntity);


GraphicsEntity::GraphicsEntity(const dMatrix &matrix, GraphicsEntity *const parent)
        : dClassInfo(), dHierarchy<GraphicsEntity>(), m_matrix(matrix), m_curPosition(matrix.m_posit),
          m_nextPosition(matrix.m_posit), m_curRotation(dQuaternion(matrix)), m_nextRotation(dQuaternion(matrix)),
          m_meshMatrix(dGetIdentityMatrix()), m_mesh(NULL), m_userData(NULL), m_lock(0) {
    if (parent) {
        Attach(parent);
    }
}


GraphicsEntity::GraphicsEntity(GraphicsManager &world, const dScene *const scene, dScene::dTreeNode *const rootSceneNode,
                       dTree<GraphicsMeshInterface *, dScene::dTreeNode *> &meshCache,
                       GraphicsManager::EntityDictionary &entityDictionary, GraphicsEntity *const parent)
        : dClassInfo(), dHierarchy<GraphicsEntity>(), m_matrix(dGetIdentityMatrix()), m_curPosition(0.0f, 0.0f, 0.0f, 1.0f),
          m_nextPosition(0.0f, 0.0f, 0.0f, 1.0f), m_curRotation(1.0f, 0.0f, 0.0f, 0.0f),
          m_nextRotation(1.0f, 0.0f, 0.0f, 0.0f), m_meshMatrix(dGetIdentityMatrix()), m_mesh(NULL), m_userData(NULL),
          m_lock(0) {
    // add this entity to the dictionary
    entityDictionary.Insert(this, rootSceneNode);

    // if this is a child mesh set it as child of the entity
    if (parent) {
        Attach(parent);
        dAssert (scene->FindParentByType(rootSceneNode, dSceneNodeInfo::GetRttiType()));
    }

    dSceneNodeInfo *const sceneInfo = (dSceneNodeInfo *) scene->GetInfoFromNode(rootSceneNode);
    dMatrix matrix(sceneInfo->GetTransform());
    ResetMatrix(world, matrix);
    SetNameID(sceneInfo->GetName());

    // if this node has a mesh, find it and attach it to this entity
    dScene::dTreeNode *const meshNode = scene->FindChildByType(rootSceneNode, dMeshNodeInfo::GetRttiType());
    if (meshNode) {
        GraphicsMeshInterface *const mesh = meshCache.Find(meshNode)->GetInfo();
        SetMesh(mesh, sceneInfo->GetGeometryTransform());
    }

    // we now scan for all dSceneNodeInfo node with direct connection to this rootSceneNode,
    // and we load the as children of this entity
    for (void *child = scene->GetFirstChildLink(rootSceneNode); child; child = scene->GetNextChildLink(rootSceneNode,
                                                                                                       child)) {
        dScene::dTreeNode *const node = scene->GetNodeFromLink(child);
        dNodeInfo *const info = scene->GetInfoFromNode(node);
        if (info->IsType(dSceneNodeInfo::GetRttiType())) {
            new GraphicsEntity(world, scene, node, meshCache, entityDictionary, this);
        }
    }
}

GraphicsEntity::GraphicsEntity(const GraphicsEntity &copyFrom)
        : dClassInfo(), dHierarchy<GraphicsEntity>(copyFrom), m_matrix(copyFrom.m_matrix),
          m_curPosition(copyFrom.m_curPosition), m_nextPosition(copyFrom.m_nextPosition),
          m_curRotation(copyFrom.m_curRotation), m_nextRotation(copyFrom.m_nextRotation),
          m_meshMatrix(copyFrom.m_meshMatrix), m_mesh(copyFrom.m_mesh), m_userData(NULL), m_lock(0) {
    if (m_mesh) {
        m_mesh->AddRef();
    }
}

GraphicsEntity::~GraphicsEntity(void) {
    if (m_userData) {
        delete m_userData;
    }
    SetMesh(NULL, dGetIdentityMatrix());
}


dBaseHierarchy *GraphicsEntity::CreateClone() const {
    return new GraphicsEntity(*this);
}


GraphicsEntity::UserData *GraphicsEntity::GetUserData() {
    return m_userData;
}

void GraphicsEntity::SetUserData(UserData *const data) {
    m_userData = data;
}

void GraphicsEntity::TransformCallback(const NewtonBody *const body, const dFloat *const matrix, int threadIndex) {
    GraphicsEntity *const ent = (GraphicsEntity *) NewtonBodyGetUserData(body);

    if (ent) {
        GraphicsManager *const scene = (GraphicsManager *) NewtonWorldGetUserData(NewtonBodyGetWorld(body));
        dMatrix transform(matrix);
        dQuaternion rot(transform);
        ent->SetMatrix(*scene, rot, transform.m_posit);
    }
}

void GraphicsEntity::SetMesh(GraphicsMeshInterface *const mesh, const dMatrix &meshMatrix) {
    m_meshMatrix = meshMatrix;
    if (m_mesh) {
        m_mesh->Release();
    }
    m_mesh = mesh;
    if (mesh) {
        mesh->AddRef();
    }
}

dMatrix GraphicsEntity::GetCurrentMatrix() const {
    return dMatrix(m_curRotation, m_curPosition);
}

dMatrix GraphicsEntity::GetNextMatrix() const {
    return dMatrix(m_nextRotation, m_nextPosition);
}

dMatrix GraphicsEntity::CalculateGlobalMatrix(const GraphicsEntity *const root) const {
    dMatrix matrix(dGetIdentityMatrix());
    for (const GraphicsEntity *ptr = this; ptr != root; ptr = ptr->GetParent()) {
        matrix = matrix * ptr->GetCurrentMatrix();
    }
    return matrix;
}

void GraphicsEntity::SetMatrix(GraphicsManager &world, const dQuaternion &rotation, const dVector &position) {
    // read the data in a critical section to prevent race condition from other thread
    world.Lock(m_lock);

    m_curPosition = m_nextPosition;
    m_curRotation = m_nextRotation;

    m_nextPosition = position;
    m_nextRotation = rotation;

    dFloat angle = m_curRotation.DotProduct(m_nextRotation);
    if (angle < 0.0f) {
        m_curRotation.Scale(-1.0f);
    }

    // release the critical section
    world.Unlock(m_lock);
}

void GraphicsEntity::ResetMatrix(GraphicsManager &world, const dMatrix &matrix) {
    dQuaternion rot(matrix);
    SetMatrix(world, rot, matrix.m_posit);
    SetMatrix(world, rot, matrix.m_posit);
    InterpolateMatrix(world, 0.0f);
}

void GraphicsEntity::InterpolateMatrix(GraphicsManager &world, dFloat param) {
    // read the data in a critical section to prevent race condition from other thread
    world.Lock(m_lock);

    dVector p0(m_curPosition);
    dVector p1(m_nextPosition);
    dQuaternion r0(m_curRotation);
    dQuaternion r1(m_nextRotation);

    // release the critical section
    world.Unlock(m_lock);

    dVector posit(p0 + (p1 - p0).Scale(param));
    dQuaternion rotation(r0.Slerp(r1, param));

    m_matrix = dMatrix(rotation, posit);

    if (m_userData) {
        m_userData->OnInterpolateMatrix(world, param);
    }
}

void GraphicsEntity::Render(dFloat timestep, GraphicsManager *const scene) const {
    // save the model matrix before changing it Matrix
    glPushMatrix();

    // Set The matrix for this entity Node
    glMultMatrix(&m_matrix[0][0]);

    // Render mesh if there is one
    if (m_mesh) {
        glPushMatrix();
        glMultMatrix(&m_meshMatrix[0][0]);
        m_mesh->Render(scene);
//		m_mesh->RenderNormals ();

        if (m_userData) {
            m_userData->OnRender(timestep);
        }
        glPopMatrix();
    }

    for (GraphicsEntity *child = GetChild(); child; child = child->GetSibling()) {
        child->Render(timestep, scene);
    }

    // restore the matrix before leaving
    glPopMatrix();

}

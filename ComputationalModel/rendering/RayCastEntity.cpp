//
// Created by Adrian Ionita on 01/08/2016.
//

#include "Util.h"
#include "GraphicsEntity.h"
#include "RayCastEntity.h"
#include <vector>
#include <array>

using namespace std;

RayCastEntity::RayCastEntity(vector<dVector> *startPoints, vector<dVector> *intersectionPoints)
    : GraphicsEntity(dGetIdentityMatrix(), NULL) {
    this->startPoints = startPoints;
    this->intersectionPoints = intersectionPoints;
}

void RayCastEntity::Render(dFloat timeStep, GraphicsManager *const scene) const{
    //draw raycast and intersection points
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //intersection points
    glColor4f(1.0f, 0.0f, 0.0f,0.4f);
    glPointSize(6.0f);
    glBegin(GL_POINTS);

    unsigned startIdx = indices && indices->at(0) != -1  ? indices->at(0) : 0 ;
    if(indices && indices->at(0) != -1)
        for (int i : *indices) {
            glVertex3f(intersectionPoints->at(i).m_x, intersectionPoints->at(i).m_y, intersectionPoints->at(i).m_z);
        }
    else
        for (auto i = intersectionPoints->size() - 1; i > 0; i--) {
            glVertex3f(intersectionPoints->at(i).m_x, intersectionPoints->at(i).m_y, intersectionPoints->at(i).m_z);
        }

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f); //mark starting point for visual feedback
    glVertex3f(intersectionPoints->at(startIdx).m_x, intersectionPoints->at(startIdx).m_y, intersectionPoints->at(startIdx).m_z);

    glEnd();
    glPointSize(1.0f);
    glColor3f(1.0f, 1.0f, 1.0f);

    if (!startPoints) return;

    //raycast lines
    glColor4f(0.0f, 0.7f, 0.7f, 0.3f);
    glBegin(GL_LINES);

    if(indices && indices->at(0) != -1)
        for (int i : *indices) {
            glVertex3f(startPoints->at(i).m_x, startPoints->at(i).m_y, startPoints->at(i).m_z);
            glVertex3f(intersectionPoints->at(i).m_x, intersectionPoints->at(i).m_y, intersectionPoints->at(i).m_z);
        }
    else
        for (int i = startPoints->size() - 1; i >= 0; i--) {
            glVertex3f(startPoints->at(i).m_x, startPoints->at(i).m_y, startPoints->at(i).m_z);
            glVertex3f(intersectionPoints->at(i).m_x, intersectionPoints->at(i).m_y, intersectionPoints->at(i).m_z);
        }

    glDisable(GL_BLEND);
    glEnd();
}

void RayCastEntity::SetSubSurface(array<int, VIEW_DIMENSION * VIEW_DIMENSION> *indices) {
    this->indices = indices;
}



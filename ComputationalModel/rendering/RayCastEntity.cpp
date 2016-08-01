//
// Created by Adrian Ionita on 01/08/2016.
//

#include "Util.h"
#include "GraphicsEntity.h"
#include "RayCastEntity.h"

RayCastEntity::RayCastEntity(const dVector startPoints[][VIEW_DIMENSION], const dVector intersectionPoints[][VIEW_DIMENSION])
    : GraphicsEntity(dGetIdentityMatrix(), NULL) {
    this->startPoints = startPoints;
    this->intersectionPoints = intersectionPoints;
}

void RayCastEntity::Render(dFloat timeStep, GraphicsManager *const scene) const{
    //draw raycast and intersection points
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    //raycast lines
//    glColor3f(0.0f, 0.7f, 0.7f);
//    glBegin(GL_LINES);
//    for (int i = 0; i < VIEW_DIMENSION; i++) {
//        for (int j = 0; j < VIEW_DIMENSION; j++) {
//            glVertex3f(startPoints[i][j].m_x, startPoints[i][j].m_y, startPoints[i][j].m_z);
//            glVertex3f(intersectionPoints[i][j].m_x, intersectionPoints[i][j].m_y, intersectionPoints[i][j].m_z);
//        }
//    }
//    glEnd();

    //intersection points
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(6.0f);
    glBegin(GL_POINTS);
    for (int i = 0; i < VIEW_DIMENSION; i++) {
        for (int j = 0; j < VIEW_DIMENSION; j++) {
            glVertex3f(intersectionPoints[i][j].m_x, intersectionPoints[i][j].m_y, intersectionPoints[i][j].m_z);
        }
    }
    glEnd();
    glPointSize(1.0f);

    glColor3f(1.0f, 1.0f, 1.0f);
}


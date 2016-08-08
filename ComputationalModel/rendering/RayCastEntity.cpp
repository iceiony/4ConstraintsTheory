//
// Created by Adrian Ionita on 01/08/2016.
//

#include "Util.h"
#include "GraphicsEntity.h"
#include "RayCastEntity.h"
#include <vector>
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
    glColor4f(1.0f, 0.0f, 0.0f,0.6f);
    glPointSize(6.0f);
    glBegin(GL_POINTS);

    for (int i = startPoints->size() - 1; i >= 0; i--) {
        if(i==0){
            glColor4f(0.0f, 0.0f, 1.0f, 1.0f); //mark starting point for visual feedback
        }

        glVertex3f(intersectionPoints->at(i).m_x, intersectionPoints->at(i).m_y, intersectionPoints->at(i).m_z);
    }

    glEnd();
    glPointSize(1.0f);

    glColor3f(1.0f, 1.0f, 1.0f);

    //raycast lines
    glColor4f(0.0f, 0.7f, 0.7f, 0.9f);
    glBegin(GL_LINES);

    for (int i = startPoints->size() - 1; i >= 0; i--) {
        glVertex3f(startPoints->at(i).m_x, startPoints->at(i).m_y, startPoints->at(i).m_z);
        glVertex3f(intersectionPoints->at(i).m_x, intersectionPoints->at(i).m_y, intersectionPoints->at(i).m_z);
    }

    glDisable(GL_BLEND);
    glEnd();


}


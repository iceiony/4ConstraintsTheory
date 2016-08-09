//
// Created by Adrian Ionita on 01/08/2016.
//
#include "Util.h"
#include "GraphicsEntity.h"
#include "GraphicsManager.h"
#include <vector>
#include <array>

using namespace std;


#ifndef MATLABNEWTONDYNAMICS_RAYCASTENTITY_H
#define MATLABNEWTONDYNAMICS_RAYCASTENTITY_H
class RayCastEntity: public GraphicsEntity {
private:
    vector<dVector> *startPoints;
    vector<dVector> *intersectionPoints;
public:
    RayCastEntity(vector<dVector> *startPoints, vector<dVector> *intersectionPoints);
    virtual void Render(dFloat timeStep, GraphicsManager *const scene) const;

    void SetSubSurface(array<int, VIEW_DIMENSION * VIEW_DIMENSION> *indices);

    array<int, VIEW_DIMENSION * VIEW_DIMENSION> *indices = nullptr;
};
#endif //MATLABNEWTONDYNAMICS_RAYCASTENTITY_H

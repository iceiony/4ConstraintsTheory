//
// Created by Adrian Ionita on 01/08/2016.
//

#ifndef MATLABNEWTONDYNAMICS_RAYCASTENTITY_H
#define MATLABNEWTONDYNAMICS_RAYCASTENTITY_H


class RayCastEntity: public GraphicsEntity {
private:
    const dVector (*startPoints)[VIEW_DIMENSION];
    const dVector (*intersectionPoints)[VIEW_DIMENSION];
public:
    RayCastEntity(const dVector startPoints[][VIEW_DIMENSION], const dVector intersectionPoints[][VIEW_DIMENSION]);
    virtual void Render(dFloat timeStep, GraphicsManager *const scene) const;
};
#endif //MATLABNEWTONDYNAMICS_RAYCASTENTITY_H

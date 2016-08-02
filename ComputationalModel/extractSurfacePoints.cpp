#include <iomanip>
#include "Util.h"
#include "Simulation.h"
#include "GraphicsEntity.h"
#include "RayCastEntity.h"

#define OUTPUT_FILE "./surfaces/surface.csv"

static dFloat RayCast (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam) {
    dFloat *const paramPtr = (dFloat *) userData;
    if (intersetParam < paramPtr[0]) {
        paramPtr[0] = intersetParam;
    }
    return paramPtr[0];
}

static void ExportSurface(const dVector intersectionPoints[][VIEW_DIMENSION]){
    //output points from current view
    std::ofstream outFile;
    outFile.open(OUTPUT_FILE);
    for(int i=0;i<VIEW_DIMENSION;i++) {
        for (int j = 0; j < VIEW_DIMENSION; j++) {
            dVector point = intersectionPoints[i][j];
            outFile << point.m_x << ',' << point.m_y << ',' << point.m_z << "\n";
        }
    }
    outFile.flush();
    outFile.close();

    std::cout<< "Surface exported\n";
}

int main(int argc, char * argv[]) {

    if(argc == 0)
    {
        std::cout << "No model file name given as parameter";
        return -1;
    }
    const char * fileName = argv[0];

    std::cout << "Loading model from :" << fileName;

    Simulation sim(nullptr);
    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);

    NewtonBody *modelBody = sim.LoadModel("obj51.3ds");

    //set object position and rotation
    dMatrix origin(dGetIdentityMatrix());
    origin.m_posit = dVector( .0f, 1.0f, .0f);
    NewtonBodySetMatrix(modelBody,&origin[0][0]);

    graphicsManager.Register(modelBody, GRAY);
    graphicsManager.SetCamera(dVector(-.1f, 3.1f, .0f), -80, 0);

    //setup raycast parameters
    dVector startPoints[VIEW_DIMENSION][VIEW_DIMENSION];
    dVector endPoints[VIEW_DIMENSION][VIEW_DIMENSION];
    dVector intersectionPoints[VIEW_DIMENSION][VIEW_DIMENSION];

    float mid = (float) VIEW_DIMENSION / 2.0f;
    for(int i=0;i<VIEW_DIMENSION;i++){
        for(int j=0;j<VIEW_DIMENSION;j++){
            startPoints[i][j].m_x = (i-mid)*CAST_STEP;
            startPoints[i][j].m_y = 3.0f;
            startPoints[i][j].m_z = (j-mid)*CAST_STEP;
//            startPoints[i][j] = dVector(.0f,2.5f,.0f);

            endPoints[i][j].m_x = (i - mid)*CAST_STEP;
            endPoints[i][j].m_y = 0.0f;
            endPoints[i][j].m_z = (j - mid)*CAST_STEP;
        }
    }

    RayCastEntity *castEntity = new RayCastEntity(startPoints,intersectionPoints);
    graphicsManager.Append(castEntity);

    bool isFirstPause = false;

    while (!sim.IsFinished()) {
        if (!graphicsManager.IsPhysicsPaused()) {
            isFirstPause = true;

            // update the the state of all bodies in the scene
            NewtonBodySetOmega(modelBody, &dVector(.5f,.5f,.5f)[0]);
            sim.UpdatePhysics();

            //calculate intersection points
            for(int i=0;i<VIEW_DIMENSION;i++) {
                for (int j = 0; j < VIEW_DIMENSION; j++) {
                    dFloat scaleParam(1.1f);
                    NewtonWorldRayCast(sim.GetNewtonWorld(), &startPoints[i][j][0], &endPoints[i][j][0], RayCast, &scaleParam, NULL, 1);
                    intersectionPoints[i][j] = startPoints[i][j] + (endPoints[i][j] - startPoints[i][j]).Scale (scaleParam);
                }
            }
        }
        else {

            //first frame after pause save the current surface
            if (isFirstPause) {
                isFirstPause = false;
                ExportSurface(intersectionPoints);
            }
        }

        if(!graphicsManager.IsWindowClosed())
            graphicsManager.UpdateGraphics(sim.GetTimeStep());
        else break;

    }

}

#include <iomanip>
#include "Util.h"
#include "Simulation.h"
#include "GraphicsEntity.h"
#include "RayCastEntity.h"

#define OUTPUT_FILE "./surfaces/surface_tool"

static dFloat RayCast (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam) {
    dFloat *const paramPtr = (dFloat *) userData;
    if (intersetParam < paramPtr[0]) {
        paramPtr[0] = intersetParam;
    }
    return paramPtr[0];
}

static void ExportSurface(const dVector intersectionPoints[][VIEW_DIMENSION],int suffixCount){
    //output points from current view
    std::ostringstream fileName;
    fileName << OUTPUT_FILE << suffixCount << ".csv";

    std::ofstream outFile;
    outFile.open(fileName.str());

    for(int i=0;i<VIEW_DIMENSION;i++) {
        for (int j = 0; j < VIEW_DIMENSION; j++) {
            dVector point = intersectionPoints[i][j];
            outFile << point.m_x << ',' << point.m_y << ',' << point.m_z << "\n";
        }
    }
    outFile.flush();
    outFile.close();

    std::cout<< "Surface exported to " << fileName.str() << "\n";
}

void UpdateRayCastPosition(dVector startPoints[][VIEW_DIMENSION]
        ,dVector endPoints[][VIEW_DIMENSION]
        ,dMatrix positionOffset) {

//    dVector position = positionOffset.m_front;
//    std::cout << position.m_x << ' ' << position.m_y << ' ' << position.m_z  << "\n\r";
//    position = positionOffset.m_posit;

    float mid = (float) VIEW_DIMENSION / 2.0f;
    for(int i=0;i<VIEW_DIMENSION;i++){
        for(int j=0;j<VIEW_DIMENSION;j++){
            float rotationFactor = positionOffset.m_posit.m_y * 1.57;

            startPoints[i][j].m_x = (i-mid)*CAST_STEP + positionOffset.m_posit.m_x;
            startPoints[i][j].m_y = positionOffset.m_posit.m_y;
            startPoints[i][j].m_z = (j-mid)*CAST_STEP + positionOffset.m_posit.m_z;

            endPoints[i][j].m_x = (i - mid)*CAST_STEP + positionOffset.m_posit.m_x + rotationFactor * positionOffset.m_front.m_x;
            endPoints[i][j].m_y = -0.1f;
            endPoints[i][j].m_z = (j - mid)*CAST_STEP + positionOffset.m_posit.m_z + rotationFactor * positionOffset.m_front.m_z;
        }
    }
}

int main(int argc, char * argv[]) {

    if(argc == 1)
    {
        std::cout << "No model file name given as parameter \n\r";
        return -1;
    }
    const char * fileName = argv[1];

    std::cout << "Loading model from :" << fileName << "\n\r";

    Simulation sim(nullptr);
    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);

    NewtonBody *modelBody = sim.LoadModel(fileName);

    //set object position and rotation
    dMatrix origin(dGetIdentityMatrix()*dPitchMatrix(90.0f*3.14f/180.0f)*dYawMatrix(-90.0f*3.14f/180.0f));
    origin.m_posit = dVector( -.0f, 1.0f, -.0f);
    NewtonBodySetMatrix(modelBody,&origin[0][0]);

    graphicsManager.Register(modelBody, GRAY);
    graphicsManager.SetCamera(dVector(0.f, 3.5f, .0f), -90, 0);

    //setup raycast parameters
    dVector startPoints[VIEW_DIMENSION][VIEW_DIMENSION];
    dVector endPoints[VIEW_DIMENSION][VIEW_DIMENSION];
    dVector intersectionPoints[VIEW_DIMENSION][VIEW_DIMENSION];

    RayCastEntity *castEntity = new RayCastEntity(startPoints,intersectionPoints);
    graphicsManager.Append(castEntity);

    bool isFirstPause = false;
    int outCount = 0;

    while (!sim.IsFinished()) {
        if (!graphicsManager.IsPhysicsPaused()) {
            isFirstPause = true;

            // update the the state of all bodies in the scene
            NewtonBodySetOmega(modelBody, &dVector(.5f, .0f, .0f)[0]);
            sim.UpdatePhysics();
        }

        UpdateRayCastPosition(startPoints,endPoints,graphicsManager.GetCameraMatrix());

        //calculate intersection points
        for(int i=0;i<VIEW_DIMENSION;i++) {
            for (int j = 0; j < VIEW_DIMENSION; j++) {
                dFloat scaleParam(1.1f);
                NewtonWorldRayCast(sim.GetNewtonWorld(), &startPoints[i][j][0], &endPoints[i][j][0], RayCast, &scaleParam, NULL, 1);
                intersectionPoints[i][j] = startPoints[i][j] + (endPoints[i][j] - startPoints[i][j]).Scale (scaleParam);
            }
        }

        if (graphicsManager.IsPhysicsPaused() && isFirstPause){
            isFirstPause = false;
            outCount++;
            ExportSurface(intersectionPoints,outCount);
        }

        if(!graphicsManager.IsWindowClosed())
            graphicsManager.UpdateGraphics(sim.GetTimeStep());
        else break;

    }

}

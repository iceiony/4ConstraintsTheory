#include <iomanip>
#include "Util.h"
#include "ExhaustiveSimulation.h"
#include "GraphicsEntity.h"
#include "RayCastEntity.h"
#include "PhysicsUtils.h"

#include <vector>
#include <string>
#include <sstream>

using namespace std;

#define OUTPUT_FILE "./surfaces/surface_obj"

static void UpdateRayCastPosition(vector<dVector> *startPoints, vector<dVector> *endPoints, dMatrix positionOffset)
{
//    dVector position = positionOffset.m_front;
//    std::cout << position.m_x << ' ' << position.m_y << ' ' << position.m_z  << "\n\r";
//    position = positionOffset.m_posit;

    float mid = (float) VIEW_DIMENSION / 2.0f;
    for(int i=0;i<VIEW_DIMENSION;i++){
        for(int j=0;j<VIEW_DIMENSION;j++){

            auto idx = (unsigned long) i * VIEW_DIMENSION + j;
            float rotationFactor = positionOffset.m_posit.m_y * 1.57f;

            startPoints->at(idx).m_x = (i-mid)*CAST_STEP + positionOffset.m_posit.m_x;
            startPoints->at(idx).m_y = positionOffset.m_posit.m_y;
            startPoints->at(idx).m_z = (j-mid)*CAST_STEP + positionOffset.m_posit.m_z;

            endPoints->at(idx).m_x = (i - mid)*CAST_STEP + positionOffset.m_posit.m_x + rotationFactor * positionOffset.m_front.m_x;
            endPoints->at(idx).m_y = -0.1f;
            endPoints->at(idx).m_z = (j - mid)*CAST_STEP + positionOffset.m_posit.m_z + rotationFactor * positionOffset.m_front.m_z;
        }
    }
}

static void ExportSurface(vector<dVector> *intersectionPoints, int suffixCount){
    //output points from current view
    std::ostringstream fileName;
    fileName << OUTPUT_FILE << suffixCount << ".csv";

    std::ofstream outFile;
    outFile.open(fileName.str());

    for(unsigned int i=0;i<VIEW_DIMENSION;i++) {
        for (unsigned int j = 0; j < VIEW_DIMENSION; j++) {
            dVector point = intersectionPoints->at( i * VIEW_DIMENSION + j);
            outFile << point.m_x << ',' << point.m_y << ',' << point.m_z << "\n";
        }
    }
    outFile.flush();
    outFile.close();

    std::cout<< "Surface exported to " << fileName.str() << "\n";
}

int main(int argc, char * argv[]) {
	const char * fileName = "obj51.3ds" ;

    if(argc == 1)
        std::cout << "No model file name given as parameter (using default)\n\r";
	else 
		fileName = argv[1];
    
	std::cout << "Loading model from :" << fileName << "\n\r";

    ExhaustiveSimulation sim(nullptr);
    NewtonBody *modelBody = sim.LoadModel(fileName);

    //set object position and rotation
    dMatrix origin(dGetIdentityMatrix()*dPitchMatrix(radians(90.0f))*dYawMatrix(radians(-90.0f)));
    origin.m_posit = dVector( .5f, 1.0f, -.0f);
    NewtonBodySetMatrix(modelBody,&origin[0][0]);

    //initialise graphics
    GraphicsManager graphicsManager(sim.GetNewtonWorld());
    graphicsManager.Register(sim.GetFloor(), BLUE);
    graphicsManager.Register(modelBody, GRAY);
    graphicsManager.SetCamera(dVector(0.f, 3.5f, .0f), -90, 0);

    //setup raycast parameters
    vector<dVector> *startPoints = new vector<dVector>(VIEW_DIMENSION * VIEW_DIMENSION);
    vector<dVector> *endPoints = new vector<dVector>(VIEW_DIMENSION * VIEW_DIMENSION);
    vector<dVector> *intersectionPoints = new vector<dVector>(VIEW_DIMENSION * VIEW_DIMENSION);

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
                auto idx = (unsigned int) i * VIEW_DIMENSION + j;
                dFloat scaleParam(1.1f);
                NewtonWorldRayCast(sim.GetNewtonWorld(), &startPoints->at(idx)[0], &endPoints->at(idx)[0], RayCast, &scaleParam, NULL, 1);
                intersectionPoints->at(idx) = startPoints->at(idx) + (endPoints->at(idx) - startPoints->at(idx)).Scale (scaleParam);
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

    delete(startPoints);
    delete(endPoints);
    delete(intersectionPoints);

}


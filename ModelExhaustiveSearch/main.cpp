#include <iostream>
#include "Util.h"
#include "DemoEntityManager.h"
#include <GLFW/glfw3.h>

void SimpleConvexApproximation(DemoEntityManager* const scene);

using namespace std;
int main() {

    //init graphics
    if(!glfwInit()){
        cerr << "Could not init GLFW \n";
        return -1;
    };

    GLFWwindow *window = glfwCreateWindow(800,600,"Simulation",NULL,NULL);

    if(!window){
        cerr << "Could not open OpenGL window \n";
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    // init rendering entities
    DemoEntityManager *scene = new DemoEntityManager();
    SimpleConvexApproximation(scene);


    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);
                                                   
        scene->RenderFrame();
        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }


    glfwTerminate();
    return 0;
}


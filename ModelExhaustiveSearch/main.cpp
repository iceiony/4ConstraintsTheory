#include <iostream>
//#include <Newton.h>
//#include <dVector.h>
//#include <dMatrix.h>
#include <GLFW/glfw3.h>

using namespace std;

int main() {

    if(!glfwInit()){
        cerr << "Could not init GLFW \n";
        return -1;
    };


    GLFWwindow *window = glfwCreateWindow(600,480,"Simulation",NULL,NULL);

    if(!window){
        cerr << "Could not open OpenGL window \n";
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }


    glfwTerminate();
    return 0;
}
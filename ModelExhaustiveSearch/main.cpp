#include <iostream>
#include "Util.h"
#include "DemoEntityManager.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

void SimpleConvexApproximation(DemoEntityManager *const scene);

using namespace std;

class Simulation {

private:
    GLFWwindow *window;

    static DemoEntityManager *scene;

    static void WindowResizeCallback(GLFWwindow *window, int width, int height) {
        scene->SetWindowSize(width, height);
    }

public:
    Simulation() {
        if (!glfwInit()) {
            throw "Could not init GLFW \n";

        };

        window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Simulation", NULL, NULL);

        if (!window) {
            throw "Could not open OpenGL window \n";
        }

        /* Make the window's context current */
        glfwMakeContextCurrent(window);

        /* Init rendering entities */
        scene = new DemoEntityManager(window);
        scene->SetWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);

        glfwSetWindowSizeCallback(window, WindowResizeCallback);
    }

    ~Simulation() {
        glfwTerminate();
    }

    void RunSimulation() {

        // init rendering entities
        SimpleConvexApproximation(scene);


        /* Loop until the user closes the window */
        while (!glfwWindowShouldClose(window)) {
            /* Render here */
            glClear(GL_COLOR_BUFFER_BIT);

            scene->RenderFrame();
            /* Swap front and back buffers */
            glfwSwapBuffers(window);

            /* Poll for and process events */
            glfwPollEvents();
        }
    }


};

DemoEntityManager *Simulation::scene;

int main() {
    Simulation sim;
    sim.RunSimulation();
    return 0;
}


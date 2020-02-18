#include "geometry.hpp"
#define GLFW_INCLUDE_NONE
#include "hull.hpp"
#include "model.hpp"
#include "shader.hpp"
#include "window.hpp"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <glad/glad.h>
#include <iostream>
#include <random>
#include <vector>

constexpr auto const vertCode = R"END(
    #version 430

    layout (location = 0) in vec3 vertex;
    layout (location = 0) uniform mat4 pMatrix;
    layout (location = 4) uniform mat4 vMatrix;
    layout (location = 8) uniform mat4 mMatrix;

    void main() {
        gl_Position = pMatrix * vMatrix * mMatrix * vec4(vertex, 1.0);
        gl_PointSize = 10.0;
    }
)END";

constexpr auto const fragCode = R"END(
    #version 430

    layout (location = 0) out vec3 fragColor;
    layout (location = 12) uniform vec3 color;

    void main() {
        fragColor = color;
    }
)END";

/** Report an error and shutdown. */
void error_shutdown(const char* errorMsg) {
    std::cout << errorMsg;
    glfwTerminate();
    exit(-1);
}

int main() {
    // Init GLFW
    if (glfwInit() != GLFW_TRUE)
        error_shutdown("Failed to initialize GLFW\n");

    // Create Window
    const Window window(512, 512);
    if (!window.exists())
        error_shutdown("Failed to create a window.\n");

    // Init GL functions
    glfwMakeContextCurrent(window.pointer());
    if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0)
        error_shutdown("Failed to initialize OpenGL context.\n");

    // Make shaders
    const Shader shader(vertCode, fragCode);
    if (!shader.valid()) {
        const auto errorLog(shader.errorLog());
        error_shutdown(errorLog.c_str());
    }

    // Make models
    const auto pointCloud = Hull::generate_point_cloud(7.5F, 256);
    const Model hullModel(Hull::generate_convex_hull(pointCloud));
    const Model cloudModel(pointCloud);

    // Enable point rendering
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Main Loop
    double lastTime(0.0F);
    double rotation(0.0F);
    while (glfwWindowShouldClose(window.pointer()) == 0) {
        const auto time = glfwGetTime();
        rotation += (time - lastTime) * 2.5F;

        // Bind shader
        shader.bind();

        // Set matrix
        const auto distance = 16.0F;
        const auto pi = 3.14159F;
        const auto pMatrix = mat4::perspective(1.5708F, 1.0F, 0.01F, 10.0F);
        const auto vMatrix = mat4::lookAt(
            vec3{ distance * sinf(static_cast<float>(rotation) / pi), 0,
                  distance * cosf(static_cast<float>(rotation) / pi) },
            vec3{ 0, 0, 0 }, vec3{ 0, 1, 0 });
        const auto mMatrix =
            mat4{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
        shader.uniformLocation(0, pMatrix);
        shader.uniformLocation(4, vMatrix);
        shader.uniformLocation(8, mMatrix);
        shader.uniformLocation(12, vec3{ 1, 1, 1 });

        // Draw hull
        glClear(GL_COLOR_BUFFER_BIT);
        hullModel.bind();
        hullModel.draw(GL_TRIANGLES);

        // Bind point cloud model and draw
        shader.uniformLocation(12, vec3{ 1, 0, 0 });
        cloudModel.bind();
        cloudModel.draw(GL_POINTS);

        lastTime = time;
        glfwPollEvents();
        glfwSwapBuffers(window.pointer());
    }

    // Success
    glfwTerminate();
    return 0;
}
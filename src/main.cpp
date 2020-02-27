#define GLFW_INCLUDE_NONE
#include "hull.hpp"
#include "mat.hpp"
#include "model.hpp"
#include "shader.hpp"
#include "vec.hpp"
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

    layout (location = 0) out vec4 fragColor;
    layout (location = 12) uniform vec4 color;

    void main() {
        fragColor = color;
    }
)END";

/** Report an error and shutdown. */
void error_shutdown(const std::string& errorMsg) {
    std::cout << errorMsg;
    glfwTerminate();
    exit(-1);
}

void render_loop_func(
    const double& deltaTime, double& rotation, const Shader& shader,
    const Model& hullModel, const Model& cloudModel) noexcept {
    // Update rotation based on deltaTime
    rotation += deltaTime * 2.5F;

    // Calculate viewing perspective and matrices
    constexpr auto distance = 16.0F;
    constexpr auto math_pi = 3.14159F;
    const auto pMatrix = mat4::perspective(1.5708F, 1.0F, 0.01F, 10.0F);
    const auto vMatrix = mat4::lookAt(
        vec3{ distance * sinf(static_cast<float>(rotation) / math_pi), 0,
              distance * cosf(static_cast<float>(rotation) / math_pi) },
        vec3{ 0, 0, 0 }, vec3{ 0, 1, 0 });
    const auto mMatrix = mat4();

    // Flush buffers and set starting parameters
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);

    // Draw Gray Hull Back-face
    glBlendFunc(GL_ONE, GL_ZERO);
    shader.uniformLocation(0, pMatrix);
    shader.uniformLocation(4, vMatrix);
    shader.uniformLocation(8, mMatrix);
    shader.uniformLocation(12, vec4{ 0.25F });
    shader.bind();
    hullModel.bind();
    hullModel.draw(GL_TRIANGLES);

    // Draw internal point cloud model
    glDepthFunc(GL_ALWAYS);
    shader.uniformLocation(12, vec4{ 1, 0.25F, 0.25F, 1 });
    cloudModel.bind();
    cloudModel.draw(GL_POINTS);

    // Draw Hull White front-face
    glDepthFunc(GL_LEQUAL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    shader.uniformLocation(12, vec4{ 1, 1, 1, 0.25F });
    hullModel.bind();
    hullModel.draw(GL_TRIANGLES);

    // Draw triangle outline
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    shader.uniformLocation(12, vec4{ 0.2F, 0.5F, 1, 1 });
    hullModel.draw(GL_TRIANGLES);

    // Draw outside points
    shader.uniformLocation(12, vec4{ 0.5F, 1, 0.2F, 1 });
    cloudModel.bind();
    cloudModel.draw(GL_POINTS);
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
    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) ==
        0)
        error_shutdown("Failed to initialize OpenGL context.\n");

    // Make shaders
    const Shader shader(vertCode, fragCode);
    if (!shader.valid())
        error_shutdown(shader.errorLog());

    // Make models
    const auto seed(static_cast<unsigned int>(glfwGetTime()));
    const auto pointCloud(Hull::generate_point_cloud(7.5F, 512, seed));
    const Model hullModel(Hull::generate_convex_hull(pointCloud));
    const Model cloudModel(pointCloud);

    // Enable point rendering and blending
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glLineWidth(4.0F);

    // Main Loop
    double lastTime(0.0);
    double rotation(0.0);
    double timeAccumulator(0.0);
    while (glfwWindowShouldClose(window.pointer()) == 0 &&
           timeAccumulator <= 10.0) {
        const auto time = glfwGetTime();
        const auto deltaTime = time - lastTime;
        render_loop_func(deltaTime, rotation, shader, hullModel, cloudModel);
        lastTime = time;
        timeAccumulator += deltaTime;
        glfwPollEvents();
        glfwSwapBuffers(window.pointer());
    }

    // Success
    glfwTerminate();
    return 0;
}
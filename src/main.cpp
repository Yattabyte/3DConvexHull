#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>

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
    const auto& mainMode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    glfwWindowHint(GLFW_RED_BITS, mainMode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mainMode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mainMode->blueBits);
    glfwWindowHint(GLFW_ALPHA_BITS, 0);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_RESET_NOTIFICATION);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GL_TRUE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    glfwWindowHint(GLFW_MAXIMIZED, GL_TRUE);
    const auto& window = glfwCreateWindow(1, 1, "Window", nullptr, nullptr);
    if (!window)
        error_shutdown("Failed to create a window.\n");

    // Init GL functions
    glfwMakeContextCurrent(window);
    if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0)
        error_shutdown("Failed to initialize OpenGL context.\n");

    // Success
    glfwTerminate();
    return 0;
}
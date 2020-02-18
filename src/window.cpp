#include "window.hpp"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

Window::~Window() { glfwDestroyWindow(m_window); }

Window::Window(const int& width, const int& height) {
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
    glfwWindowHint(GLFW_DECORATED, GL_TRUE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    glfwWindowHint(GLFW_MAXIMIZED, GL_FALSE);
    m_window = glfwCreateWindow(width, height, "Window", nullptr, nullptr);
}

bool Window::exists() const { return m_window != nullptr; }

GLFWwindow* Window::pointer() const { return m_window; }
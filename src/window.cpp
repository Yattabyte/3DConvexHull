#include "window.hpp"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>
#include <string>

//////////////////////////////////////////////////////////////////////
/// Custom Destructor
//////////////////////////////////////////////////////////////////////

Window::~Window() { glfwDestroyWindow(m_window); }

//////////////////////////////////////////////////////////////////////
/// Custom Constructor
//////////////////////////////////////////////////////////////////////

Window::Window(const int& width, const int& height) noexcept
    : m_width(width), m_height(height) {
    const auto& mainMode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    glfwWindowHint(GLFW_RED_BITS, mainMode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mainMode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mainMode->blueBits);
    glfwWindowHint(GLFW_ALPHA_BITS, 0);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_RESET_NOTIFICATION);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GL_TRUE);
    glfwWindowHint(GLFW_DECORATED, GL_TRUE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    glfwWindowHint(GLFW_MAXIMIZED, GL_FALSE);
#ifdef DEBUG
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif
    m_window = glfwCreateWindow(width, height, "Window", nullptr, nullptr);
}

//////////////////////////////////////////////////////////////////////

Window::Window(Window&& other) noexcept
    : m_width(other.m_width), m_height(other.m_height),
      m_window(other.m_window) {
    other.m_window = nullptr;
}

//////////////////////////////////////////////////////////////////////
/// operator=
//////////////////////////////////////////////////////////////////////

Window& Window::operator=(Window&& other) noexcept {
    if (&other != this) {
        m_width = other.m_width;
        m_height = other.m_height;
        m_window = other.m_window;
        other.m_window = nullptr;
    }
    return *this;
}
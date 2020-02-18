#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP

// Forward Declarations
struct GLFWwindow;

/***/
struct Window {
    // Attributes
    GLFWwindow* m_window = nullptr;

    // (De)Constructors
    /***/
    ~Window();
    /***/
    Window(const int& width, const int& height);

    // Methods
    /***/
    bool exists() const;
    /***/
    GLFWwindow* pointer() const;
};

#endif // WINDOW_HPP
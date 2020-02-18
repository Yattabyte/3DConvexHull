#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP

// Forward Declarations
struct GLFWwindow;

/** A representation of a GLFW window object. */
struct Window {
    // Attributes
    GLFWwindow* m_window = nullptr;

    // (De)Constructors
    /** Destroy this window. */
    ~Window();
    /** Construct a window with a given size.
    @param  width   the width to make the window.
    @param  height  the height to make the window. */
    Window(const int& width, const int& height);

    // Methods
    /** Check whether or not this window exists and is valid.
    @return     true if window exists, false otherwise. */
    bool exists() const;
    /** Retrieve the underlying window object pointer.
    @return     the GLFWwindow pointer. */
    GLFWwindow* pointer() const;
};

#endif // WINDOW_HPP
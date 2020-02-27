#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP

// Forward Declarations
struct GLFWwindow;

/** A representation of a GLFW window object. */
class Window {
    public:
    // Public (De)Constructors
    /** Destroy this window. */
    ~Window();
    /** Construct a window with a given size.
    @param  width   the width to make the window.
    @param  height  the height to make the window. */
    Window(const int& width, const int& height) noexcept;
    /** Default copy constructor. */
    Window(const Window& o) = default;
    /** Default move constructor. */
    Window(Window&& o) noexcept = default;

    // Public Operators
    /** Default copy-assignment operator. */
    Window& operator=(const Window& p) = default;
    /** Default move-assignment operator. */
    Window& operator=(Window&& p) noexcept = default;

    // Public Methods
    /** Check whether or not this window exists and is valid.
    @return     true if window exists, false otherwise. */
    bool exists() const noexcept;
    /** Retrieve the underlying window object pointer.
    @return     the GLFWwindow pointer. */
    GLFWwindow* pointer() const noexcept;

    private:
    // Private Attributes
    GLFWwindow* m_window = nullptr;
};

#endif // WINDOW_HPP
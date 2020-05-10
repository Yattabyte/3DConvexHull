#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP

// Forward Declarations
struct GLFWwindow;

//////////////////////////////////////////////////////////////////////
/// \class  Window
/// \brief  A representation of a GLFW window object.
class Window {
    public:
    //////////////////////////////////////////////////////////////////////
    /// \brief  Destroy this window.
    ~Window();
    //////////////////////////////////////////////////////////////////////
    /// \brief  Construct a window with a given size.
    /// \param  width       the width to make the window.
    /// \param  height      the height to make the window.
    Window(const int& width, const int& height) noexcept;
    //////////////////////////////////////////////////////////////////////
    /// \brief  Default copy constructor.
    Window(const Window& other) = delete;
    //////////////////////////////////////////////////////////////////////
    /// \brief  Default move constructor.
    Window(Window&& other) noexcept;

    //////////////////////////////////////////////////////////////////////
    /// \brief  Default copy-assignment operator.
    Window& operator=(const Window& other) = delete;
    //////////////////////////////////////////////////////////////////////
    /// \brief  Default move-assignment operator.
    Window& operator=(Window&& other) noexcept;

    //////////////////////////////////////////////////////////////////////
    /// \brief  Check whether or not this window exists and is valid.
    /// \return true if window exists, false otherwise.
    bool exists() const noexcept { return m_window != nullptr; }
    //////////////////////////////////////////////////////////////////////
    /// \brief  Retrieve the underlying window object pointer.
    /// \return the GLFWwindow pointer.
    GLFWwindow* pointer() const noexcept { return m_window; }
    //////////////////////////////////////////////////////////////////////
    /// \brief  Retrieve the window's width.
    /// \return the width of the window.
    int getWidth() const noexcept { return m_width; }
    //////////////////////////////////////////////////////////////////////
    /// \brief  Retrieve the window's height.
    /// \return the height of the window.
    int getHeight() const noexcept { return m_height; }

    private:
    int m_width = 1;                ///< The window width.
    int m_height = 1;               ///< The window height.
    GLFWwindow* m_window = nullptr; ///< The GLFW window object pointer.
};

#endif // WINDOW_HPP
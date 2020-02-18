#pragma once
#ifndef SHADER_HPP
#define SHADER_HPP

#include "geometry.hpp"
#include <string>

/** A representation of a full OpenGL shader program. */
struct Shader {
    // Attributes
    unsigned int m_vertexID = 0;
    unsigned int m_fragmentID = 0;
    unsigned int m_programID = 0;

    // (De)Constructors
    /** Destroy this shader program. */
    ~Shader();
    /** Construct a shader program.
    @param  vertexSource    the source code for the vertex shader.
    @param  fragmentSource  the source code for the fragment shader.*/
    Shader(const char* const vertexSource, const char* const fragmentSource);

    // Methods
    /** Check whether or not this program is valid.
    @return     true on success, false otherwise. */
    bool valid() const;
    /** Attempt to retrieve any error log for this program.
    @return     an error log if present. */
    std::string errorLog() const;
    /** Bind this shader to the currently active context for rendering. */
    void bind() const;
    /** Copy data to a specific uniform location.
    @param  location    the location in-shader to copy to.
    @param  vector      the data to copy-in. */
    void uniformLocation(const int& location, const vec3& vector) const;
    /** Copy data to a specific uniform location.
    @param  location    the location in-shader to copy to.
    @param  matrix      the data to copy-in. */
    void uniformLocation(const int& location, const mat4& matrix) const;
};

#endif // SHADER_HPP
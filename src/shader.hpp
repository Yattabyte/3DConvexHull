#pragma once
#ifndef SHADER_HPP
#define SHADER_HPP

#include "geometry.hpp"
#include <string>

/***/
struct Shader {
    // Attributes
    unsigned int m_vertexID = 0;
    unsigned int m_fragmentID = 0;
    unsigned int m_programID = 0;

    // (De)Constructors
    /***/
    ~Shader();
    /***/
    Shader(const char* const vertexSource, const char* const fragmentSource);

    // Methods
    /***/
    bool valid() const;
    /***/
    std::string errorLog() const;
    /***/
    void bind() const;
    /***/
    void uniformLocation(const int& location, const vec3& vector) const;
    /***/
    void uniformLocation(const int& location, const mat4& matrix) const;
};

#endif // SHADER_HPP
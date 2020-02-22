#pragma once
#ifndef MODEL_HPP
#define MODEL_HPP

#include "geometry.hpp"
#include <vector>

/** A representation of an OpenGL model. */
struct Model {
    // Attributes
    unsigned int m_vaoID = 0U;
    unsigned int m_vboID = 0U;
    size_t m_vertexCount = 0ULL;

    // (De)Constructors
    /** Destroy this model. */
    ~Model();
    /** Construct a model given a vertex set.
    @param  vertices   the vertices to use (as triangles). */
    explicit Model(const std::vector<vec3>& vertices) noexcept;
    /** Default copy constructor. */
    Model(const Model& o) = default;
    /** Default move constructor. */
    Model(Model&& o) noexcept = default;

    // Operators
    /** Default copy-assignment operator. */
    Model& operator=(const Model& p) = default;
    /** Default move-assignment operator. */
    Model& operator=(Model&& p) noexcept = default;

    // Methods
    /** Bind this model to the current context for rendering. */
    void bind() const noexcept;
    /** Draw this model.
    @param  drawMode    either GL_TRIANGLES, GL_POINTS, GL_LINES, etc. */
    void draw(const int& drawMode) const noexcept;
};

#endif // MODEL_HPP
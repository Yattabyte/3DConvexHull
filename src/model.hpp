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
    explicit Model(const std::vector<vec3>& vertices);

    // Methods
    /** Bind this model to the current context for rendering. */
    void bind() const;
    /** Draw this model..
    @param  drawMode    either GL_TRIANGLES, GL_POINTS, GL_LINES, etc. */
    void draw(const int& drawMode) const;
};

#endif // MODEL_HPP
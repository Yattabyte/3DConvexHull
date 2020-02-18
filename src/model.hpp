#ifndef MODEL_HPP
#define MODEL_HPP

#include "geometry.hpp"
#include <vector>

struct Model {
    // Attributes
    unsigned int m_vaoID = 0U;
    unsigned int m_vboID = 0U;
    size_t m_vertexCount = 0ULL;

    // (De)Constructors
    /***/
    ~Model();
    /***/
    Model(const std::vector<vec3>& verticies);

    // Methods
    /***/
    void bind() const;
    /***/
    void draw(const int& drawMode) const;
};

#endif // MODEL_HPP
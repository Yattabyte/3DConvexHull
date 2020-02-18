#include "model.hpp"
#include <glad/glad.h>

Model::~Model() {}

Model::Model(const std::vector<vec3>& verticies)
    : m_vertexCount(verticies.size()) {
    // Create GL Objects
    glCreateVertexArrays(1, &m_vaoID);
    glCreateBuffers(1, &m_vboID);

    // Load geometry into vertex buffer object
    glNamedBufferStorage(
        m_vboID, sizeof(vec3) * verticies.size(), &verticies[0],
        GL_CLIENT_STORAGE_BIT);

    // Connect and set-up the vertex array object
    glEnableVertexArrayAttrib(m_vaoID, 0);
    glVertexArrayAttribBinding(m_vaoID, 0, 0);
    glVertexArrayAttribFormat(m_vaoID, 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayVertexBuffer(m_vaoID, 0, m_vboID, 0, sizeof(vec3));
}

void Model::bind() const { glBindVertexArray(m_vaoID); }

void Model::draw(const int& drawMode) const {
    glDrawArrays(
        static_cast<GLenum>(drawMode), 0, static_cast<GLsizei>(m_vertexCount));
}
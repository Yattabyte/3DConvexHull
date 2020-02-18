#include "shader.hpp"
#include <glad/glad.h>
#include <vector>

Shader::~Shader() {}

Shader::Shader(const char* const vertexSource, const char* const fragmentSource)
    : m_vertexID(glCreateShader(GL_VERTEX_SHADER)),
      m_fragmentID(glCreateShader(GL_FRAGMENT_SHADER)),
      m_programID(glCreateProgram()) {
    // Make vertex shader
    glShaderSource(m_vertexID, 1, &vertexSource, nullptr);
    glCompileShader(m_vertexID);

    // Make fragment shader
    glShaderSource(m_fragmentID, 1, &fragmentSource, nullptr);
    glCompileShader(m_fragmentID);

    // Create program
    glAttachShader(m_programID, m_vertexID);
    glAttachShader(m_programID, m_fragmentID);

    // Link program
    glLinkProgram(m_programID);

    // Validate program
    glValidateProgram(m_programID);
    glDetachShader(m_programID, m_vertexID);
    glDetachShader(m_programID, m_fragmentID);
}

bool Shader::valid() const {
    if (m_vertexID == 0 || m_fragmentID == 0 || m_programID == 0)
        return false;

    GLint param;
    if (glGetProgramiv(m_programID, GL_LINK_STATUS, &param); param == 0)
        return false;

    return true;
}

std::string Shader::errorLog() const {
    std::string log;
    GLint param;
    if (glGetProgramiv(m_programID, GL_INFO_LOG_LENGTH, &param); param != 0) {
        std::vector<GLchar> infoLog(param);
        glGetProgramInfoLog(
            m_programID, static_cast<GLsizei>(param), nullptr, &infoLog[0]);
        log = infoLog.data();
    }
    return log;
}

void Shader::bind() const { glUseProgram(m_programID); }

void Shader::uniformLocation(const int& location, const vec3& vector) const {
    glProgramUniform3f(m_programID, location, vector.x, vector.y, vector.z);
}

void Shader::uniformLocation(const int& location, const mat4& matrix) const {
    glProgramUniformMatrix4fv(
        m_programID, location, 1, GL_FALSE, &matrix.data[0][0]);
}
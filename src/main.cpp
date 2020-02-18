#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>
#include <vector>

/** Report an error and shutdown. */
void error_shutdown(const char* errorMsg) {
    std::cout << errorMsg;
    glfwTerminate();
    exit(-1);
}

/** Make and return a window object. */
auto make_window() {
    const auto& mainMode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    glfwWindowHint(GLFW_RED_BITS, mainMode->redBits);
    glfwWindowHint(GLFW_GREEN_BITS, mainMode->greenBits);
    glfwWindowHint(GLFW_BLUE_BITS, mainMode->blueBits);
    glfwWindowHint(GLFW_ALPHA_BITS, 0);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_CONTEXT_ROBUSTNESS, GLFW_NO_RESET_NOTIFICATION);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GL_TRUE);
    glfwWindowHint(GLFW_DECORATED, GL_TRUE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_VISIBLE, GL_TRUE);
    glfwWindowHint(GLFW_MAXIMIZED, GL_FALSE);
    return glfwCreateWindow(512, 512, "Window", nullptr, nullptr);
}

constexpr auto const vertCode = R"END(
    #version 430

    layout(location = 0) in vec3 vertex;
    layout(location = 0) uniform mat4 pvmMatrix;

    void main() {
        gl_Position = vec4(vertex, 1.0);
    }
)END";

constexpr auto const fragCode = R"END(
    #version 430

    layout (location = 0) out vec3 fragColor;

    void main() {
        fragColor = vec3(1.0f);
    }
)END";

/** Make and return a shader object. */
auto make_shader() {
    // Make vertex shader
    const auto shaderVertex(glCreateShader(GL_VERTEX_SHADER));
    glShaderSource(shaderVertex, 1, &vertCode, nullptr);
    glCompileShader(shaderVertex);

    // Make fragment shader
    const auto shaderFragment(glCreateShader(GL_FRAGMENT_SHADER));
    glShaderSource(shaderFragment, 1, &fragCode, nullptr);
    glCompileShader(shaderFragment);

    // Create program
    const auto shaderProgram(glCreateProgram());
    glAttachShader(shaderProgram, shaderVertex);
    glAttachShader(shaderProgram, shaderFragment);

    // Link program
    GLint param;
    glLinkProgram(shaderProgram);
    if (glGetProgramiv(shaderProgram, GL_LINK_STATUS, &param); param == 0)
        error_shutdown("Link Failure");

    // Try to find an error
    if (glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &param); param != 0) {
        std::vector<GLchar> infoLog(param);
        glGetProgramInfoLog(
            shaderProgram, static_cast<GLsizei>(param), nullptr, &infoLog[0]);
        error_shutdown(infoLog.data());
    }

    // Validate program
    glValidateProgram(shaderProgram);
    glDetachShader(shaderProgram, shaderVertex);
    glDetachShader(shaderProgram, shaderFragment);

    return shaderProgram;
}

/** Make and return a model */
auto make_model() {
    // Create starting data
    struct vec3 {
        float x, y, z;
    };
    constexpr vec3 data[] = { { -1, -1, 0 }, { 1, -1, 0 }, { 0, 1, 0 } };
    constexpr auto size = sizeof(vec3) * 3;

    // Load geometry into vertex buffer object
    GLuint vboID(0);
    glCreateBuffers(1, &vboID);
    glNamedBufferStorage(vboID, size, &data[0], GL_CLIENT_STORAGE_BIT);

    // Connect and setup the vertex array object
    GLuint vaoID(0);
    glCreateVertexArrays(1, &vaoID);
    glEnableVertexArrayAttrib(vaoID, 0);
    glVertexArrayAttribBinding(vaoID, 0, 0);
    glVertexArrayAttribFormat(vaoID, 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayVertexBuffer(vaoID, 0, vboID, 0, sizeof(vec3));
    return vaoID;
}

int main() {
    // Init GLFW
    if (glfwInit() != GLFW_TRUE)
        error_shutdown("Failed to initialize GLFW\n");

    // Create Window
    const auto& window = make_window();
    if (!window)
        error_shutdown("Failed to create a window.\n");

    // Init GL functions
    glfwMakeContextCurrent(window);
    if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0)
        error_shutdown("Failed to initialize OpenGL context.\n");

    // Make shaders
    const auto shader = make_shader();

    // Make models
    const auto model = make_model();

    // Main Loop
    while (!glfwWindowShouldClose(window)) {
        // Bind shader
        glUseProgram(shader);

        // Bind model
        glBindVertexArray(model);

        // Draw
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    // Success
    glfwTerminate();
    return 0;
}
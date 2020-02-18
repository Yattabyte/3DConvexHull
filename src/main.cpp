#define GLFW_INCLUDE_NONE
#include "geometry.hpp"
#include "hull.hpp"
#include <GLFW/glfw3.h>
#include <cmath>
#include <glad/glad.h>
#include <iostream>
#include <random>
#include <set>
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

    layout (location = 0) in vec3 vertex;
    layout (location = 0) uniform mat4 pMatrix;
    layout (location = 4) uniform mat4 vMatrix;
    layout (location = 8) uniform mat4 mMatrix;

    void main() {
        gl_Position = pMatrix * vMatrix * mMatrix * vec4(vertex, 1.0);
        gl_PointSize = 10.0;
    }
)END";

constexpr auto const fragCode = R"END(
    #version 430

    layout (location = 0) out vec3 fragColor;
    layout (location = 12) uniform vec3 color;

    void main() {
        fragColor = color;
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

/** Generate a point cloud in 3D space, applying the scale specified. */
auto make_point_cloud(const float& scale, const size_t& numberOfPoints) {
    const std::uniform_real_distribution<float> randomFloats(-scale, scale);
    std::default_random_engine generator;
    std::vector<vec3> points(numberOfPoints);
    std::generate(std::begin(points), std::end(points), [&]() {
        return vec3{ randomFloats(generator), randomFloats(generator),
                     randomFloats(generator) };
    });
    return points;
}

/** Generate a convex hull from a set of points. */
auto make_hull(const std::vector<vec3>& points) {
    std::vector<Hull::Point3> pointsCopy;
    pointsCopy.reserve(points.size());
    for (const auto& point : points)
        pointsCopy.emplace_back(Hull::Point3(point));
    std::vector<Hull::Triangle> hull;
    if (Hull::Generate_3D(pointsCopy, hull) != 0) {
    }

    std::vector<vec3> vertices;
    vertices.reserve(hull.size() * 3);
    for (const auto& triangle : hull) {
        vertices.emplace_back(pointsCopy[triangle.a].xyz);
        vertices.emplace_back(pointsCopy[triangle.b].xyz);
        vertices.emplace_back(pointsCopy[triangle.c].xyz);
    }
    return vertices;
}

/** Make and return a model */
auto make_hull_model(const std::vector<vec3>& points) {
    // Create a convex hull from a set of points
    const auto verticies = make_hull(points);

    // Load geometry into vertex buffer object
    GLuint vboID(0);
    glCreateBuffers(1, &vboID);
    glNamedBufferStorage(
        vboID, sizeof(vec3) * verticies.size(), &verticies[0],
        GL_CLIENT_STORAGE_BIT);

    // Connect and set-up the vertex array object
    GLuint vaoID(0);
    glCreateVertexArrays(1, &vaoID);
    glEnableVertexArrayAttrib(vaoID, 0);
    glVertexArrayAttribBinding(vaoID, 0, 0);
    glVertexArrayAttribFormat(vaoID, 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayVertexBuffer(vaoID, 0, vboID, 0, sizeof(vec3));
    return std::make_pair(vaoID, verticies.size());
}

/** Make and return a model */
auto make_point_cloud_model(const std::vector<vec3>& points) {
    // Load geometry into vertex buffer object
    GLuint vboID(0);
    glCreateBuffers(1, &vboID);
    glNamedBufferStorage(
        vboID, sizeof(vec3) * points.size(), &points[0], GL_CLIENT_STORAGE_BIT);

    // Connect and set-up the vertex array object
    GLuint vaoID(0);
    glCreateVertexArrays(1, &vaoID);
    glEnableVertexArrayAttrib(vaoID, 0);
    glVertexArrayAttribBinding(vaoID, 0, 0);
    glVertexArrayAttribFormat(vaoID, 0, 3, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayVertexBuffer(vaoID, 0, vboID, 0, sizeof(vec3));
    return std::make_pair(vaoID, points.size());
}

vec3 normalize(const vec3& v) {
    float length_of_v = sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
    return vec3{ v.x / length_of_v, v.y / length_of_v, v.z / length_of_v };
}

vec3 cross(const vec3& a, const vec3& b) {
    return vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x };
}

float dot(const vec3& a, const vec3& b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

mat4 lookAt(vec3 const& eye, vec3 const& center, vec3 const& up) {
    vec3 f = normalize(center - eye);
    vec3 u = normalize(up);
    vec3 s = normalize(cross(f, u));
    u = cross(s, f);

    mat4 Result;
    Result.data[0][0] = s.x;
    Result.data[1][0] = s.y;
    Result.data[2][0] = s.z;
    Result.data[3][0] = -dot(s, eye);
    Result.data[0][1] = u.x;
    Result.data[1][1] = u.y;
    Result.data[2][1] = u.z;
    Result.data[3][1] = -dot(u, eye);
    Result.data[0][2] = -f.x;
    Result.data[1][2] = -f.y;
    Result.data[2][2] = -f.z;
    Result.data[3][2] = dot(f, eye);
    return Result;
}

mat4 perspective(
    float const& fovy, float const& aspect, float const& zNear,
    float const& zFar) {
    float const rad = fovy;
    float tanHalfFovy = tanf(rad / float(2));

    mat4 Result;
    Result.data[0][0] = 1.0F / (aspect * tanHalfFovy);
    Result.data[1][1] = 1.0F / (tanHalfFovy);
    Result.data[2][2] = -(zFar + zNear) / (zFar - zNear);
    Result.data[2][3] = -1.0F;
    Result.data[3][2] = -(2.0F * zFar * zNear) / (zFar - zNear);
    return Result;
}

int main() {
    // Init GLFW
    if (glfwInit() != GLFW_TRUE)
        error_shutdown("Failed to initialize GLFW\n");

    // Create Window
    const auto& window = make_window();
    if (window == nullptr)
        error_shutdown("Failed to create a window.\n");

    // Init GL functions
    glfwMakeContextCurrent(window);
    if (gladLoadGLLoader((GLADloadproc)glfwGetProcAddress) == 0)
        error_shutdown("Failed to initialize OpenGL context.\n");

    // Make shaders
    const auto shader = make_shader();

    // Make models
    const auto pointCloud = make_point_cloud(7.5F, 256);
    const auto& [hullModel, hullSize] = make_hull_model(pointCloud);
    const auto& [cloudModel, cloudSize] = make_point_cloud_model(pointCloud);

    // Enable point rendering
    glEnable(GL_PROGRAM_POINT_SIZE);

    // Main Loop
    double lastTime(0.0F);
    double rotation(0.0F);
    while (glfwWindowShouldClose(window) == 0) {
        const auto time = glfwGetTime();
        rotation += (time - lastTime) * 2.5F;

        // Bind shader
        glUseProgram(shader);

        // Bind hull model
        glBindVertexArray(hullModel);

        // Set matrix
        const auto distance = 16.0F;
        const auto pi = 3.14159F;
        const auto pMatrix = perspective(1.5708F, 1.0F, 0.01F, 10.0F);
        const auto vMatrix = lookAt(
            vec3{ distance * sinf(static_cast<float>(rotation) / pi), 0,
                  distance * cosf(static_cast<float>(rotation) / pi) },
            vec3{ 0, 0, 0 }, vec3{ 0, 1, 0 });
        const auto mMatrix =
            mat4{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
        glProgramUniformMatrix4fv(shader, 0, 1, GL_FALSE, &pMatrix.data[0][0]);
        glProgramUniformMatrix4fv(shader, 4, 1, GL_FALSE, &vMatrix.data[0][0]);
        glProgramUniformMatrix4fv(shader, 8, 1, GL_FALSE, &mMatrix.data[0][0]);
        glProgramUniform3f(shader, 12, 1, 1, 1);

        // Draw
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(hullSize));

        // Bind point cloud model
        glBindVertexArray(cloudModel);
        glProgramUniform3f(shader, 12, 1, 0, 0);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(cloudSize));

        lastTime = time;
        glfwPollEvents();
        glfwSwapBuffers(window);
    }

    // Success
    glfwTerminate();
    return 0;
}
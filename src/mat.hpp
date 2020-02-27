#pragma once
#ifndef MAT_HPP
#define MAT_HPP

#include "vec.hpp"

/** A 4 by 4 matrix of floats. */
class mat4 {
    public:
    // Public (De)Constructors
    /** Default destruct this vector. */
    ~mat4() = default;
    /** Default construct this vector. */
    mat4() = default;
    mat4(
        const vec4& v0, const vec4& v1, const vec4& v2, const vec4& v3) noexcept
        : m_data{ { v0.x(), v0.y(), v0.z(), v0.w() },
                  { v1.x(), v1.y(), v1.z(), v1.w() },
                  { v2.x(), v2.y(), v2.z(), v2.w() },
                  { v3.x(), v3.y(), v3.z(), v3.w() } } {}
    /** Default copy constructor. */
    mat4(const mat4& o) = default;
    /** Default move constructor. */
    mat4(mat4&& o) noexcept = default;

    // Public Operators
    /** Default copy-assignment operator. */
    mat4& operator=(const mat4& p) = default;
    /** Default move-assignment operator. */
    mat4& operator=(mat4&& p) noexcept = default;

    // Public Methods
    /** Get a pointer to the underlying data container.
    @return     pointer to the data array. */
    inline float* const data() noexcept { return &m_data[0][0]; }
    /** Get a const pointer to the underlying data container.
    @return     pointer to the data array. */
    inline const float* const data() const noexcept { return &m_data[0][0]; }
    /** Create a transform matrix looking at a point a given another point.
    @param  eye     the eye position.
    @param  center  the center of the target to look at.
    @aram   up      the up direction.
    @return         a view matrix looking at center from eye. */
    static mat4
    lookAt(const vec3& eye, const vec3& center, const vec3& up) noexcept {
        const auto f = (center - eye).normalize();
        auto u = up.normalize();
        const auto s = (f.cross(u)).normalize();
        u = s.cross(f);

        mat4 Result;
        Result.m_data[0][0] = s.x();
        Result.m_data[1][0] = s.y();
        Result.m_data[2][0] = s.z();
        Result.m_data[3][0] = -s.dot(eye);
        Result.m_data[0][1] = u.x();
        Result.m_data[1][1] = u.y();
        Result.m_data[2][1] = u.z();
        Result.m_data[3][1] = -u.dot(eye);
        Result.m_data[0][2] = -f.x();
        Result.m_data[1][2] = -f.y();
        Result.m_data[2][2] = -f.z();
        Result.m_data[3][2] = f.dot(eye);
        return Result;
    }
    /** Create a perspective projection matrix.
    @param  fovY    the vertical field of view.
    @param  aspect  the aspect ratio to use.
    @param  zNear   the near plane.
    @param  zFar    the far plane.
    @return         a perspective projection 4x4 matrix. */
    static mat4 perspective(
        float const& fovY, float const& aspect, float const& zNear,
        float const& zFar) noexcept {
        float const rad = fovY;
        float tanHalfFovy = tanf(rad / static_cast<float>(2));

        mat4 Result;
        Result.m_data[0][0] = 1.0F / (aspect * tanHalfFovy);
        Result.m_data[1][1] = 1.0F / (tanHalfFovy);
        Result.m_data[2][2] = -(zFar + zNear) / (zFar - zNear);
        Result.m_data[2][3] = -1.0F;
        Result.m_data[3][2] = -(2.0F * zFar * zNear) / (zFar - zNear);
        return Result;
    }

    private:
    // Private Attributes
    float m_data[4][4]{
        { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 }
    };
};

#endif // MAT_HPP
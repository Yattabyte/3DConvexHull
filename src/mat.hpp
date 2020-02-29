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
    /** Construct an explicit matrix of vectors, row major. */
    mat4(
        const vec4& v0, const vec4& v1, const vec4& v2, const vec4& v3) noexcept
        : m_data{ v0, v1, v2, v3 } {}
    /** Default copy constructor. */
    mat4(const mat4& o) = default;
    /** Default move constructor. */
    mat4(mat4&& o) noexcept = default;

    // Public Operators
    /** Default copy-assignment operator. */
    mat4& operator=(const mat4& p) = default;
    /** Default move-assignment operator. */
    mat4& operator=(mat4&& p) noexcept = default;
    /** Retrieve the row at the index specified.
    @param  index   the row number to retrieve.
    @return         reference to the row specified. */
    vec4& operator[](const size_t& index) { return m_data[index]; }
    /** Compare against another matrix.
    @param  o       the other matrix.
    @return         true if this equals the other matrix, false otherwise. */
    bool operator==(const mat4& o) const noexcept {
        for (int x = 0; x < 4; ++x)
            if (m_data[x] != o.m_data[x])
                return false;
        return true;
    }
    /** Compare against another matrix.
    @param  o   the other vector.
    @return     true if this doesn't equal the other matrix, false otherwise. */
    bool operator!=(const mat4& o) const noexcept { return !(*this == o); }

    // Public Methods
    /** Get a pointer to the underlying data container.
    @return         pointer to the data array. */
    float* data() noexcept { return m_data[0].data(); }
    /** Get a const pointer to the underlying data container.
    @return         pointer to the data array. */
    const float* data() const noexcept { return m_data[0].data(); }
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
        Result[0].x() = s.x();
        Result[1].x() = s.y();
        Result[2].x() = s.z();
        Result[3].x() = -s.dot(eye);
        Result[0].y() = u.x();
        Result[1].y() = u.y();
        Result[2].y() = u.z();
        Result[3].y() = -u.dot(eye);
        Result[0].z() = -f.x();
        Result[1].z() = -f.y();
        Result[2].z() = -f.z();
        Result[3].z() = f.dot(eye);
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
        Result[0].x() = 1.0F / (aspect * tanHalfFovy);
        Result[1].y() = 1.0F / (tanHalfFovy);
        Result[2].z() = -(zFar + zNear) / (zFar - zNear);
        Result[2].w() = -1.0F;
        Result[3].z() = -(2.0F * zFar * zNear) / (zFar - zNear);
        return Result;
    }

    private:
    // Private Attributes
    vec4 m_data[4]{
        { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 }
    };
};

#endif // MAT_HPP
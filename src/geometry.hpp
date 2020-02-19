#pragma once
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>

/** A vector of 3 floats. */
struct vec3 {
    // Attributes
    float x = 0.0f, y = 0.0f, z = 0.0f;

    // (De)Constructors
    /** Default destruct this vector. */
    ~vec3() = default;
    /** Default construct this vector. */
    vec3() = default;
    /** Construct a vector using 1 specific value. */
    explicit vec3(const float& value) : x(value), y(value), z(value) {}
    /** Construct a vector using 3 specific attributes.
    @param  _x  the x value to use.
    @param  _y  the y value to use.
    @param  _z  the z value to use. */
    vec3(const float& _x, const float& _y, const float& _z)
        : x(_x), y(_y), z(_z) {}
    /** Default copy constructor. */
    vec3(const vec3& o) = default;
    /** Default move constructor. */
    vec3(vec3&& o) noexcept = default;

    // Operators
    /** Default copy-assignment operator. */
    vec3& operator=(const vec3& p) = default;
    /** Default move-assignment operator. */
    vec3& operator=(vec3&& p) noexcept = default;
    /** Subtract another vector from this one.
    @param  o   the other vector.
    @return     this vector minus the other vector. */
    vec3 operator-(const vec3& o) const {
        return vec3{ x - o.x, y - o.y, z - o.z };
    }
    /** Add another vector to this one.
    @param  o   the other vector.
    @return     this vector plus the other vector. */
    vec3 operator+(const vec3& o) const {
        return vec3{ x + o.x, y + o.y, z + o.z };
    }
    /** Divide by another vector.
    @param  o   the other vector.
    @return     this vector divided by the other vector. */
    vec3 operator/(const vec3& o) const {
        return vec3{ x / o.x, y / o.y, z / o.z };
    }
    /** Compare this vector against another for sorting purposes. */
    bool operator<(const vec3& other) const {
        if (z == other.z) {
            if (x == other.x)
                return y < other.y;
            return x < other.x;
        }
        return z < other.z;
    };

    // Methods
    /** Normalize this vector.
    @return     normalized version of this vector. */
    vec3 normalize() const { return normalize(*this); }
    /** Normalize the supplied vector.
    @param  v   the vector to normalize.
    @return     normalize version of the supplied vector. */
    static vec3 normalize(const vec3& v) {
        float length_of_v = sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
        return vec3{ v.x / length_of_v, v.y / length_of_v, v.z / length_of_v };
    }
    /** Calculate the cross product of this vector.
    @param  b   the other vector to cross against.
    @return     cross product of this and the supplied vector. */
    vec3 cross(const vec3& b) const { return cross(*this, b); }
    /** Calculate the cross product the supplied vectors.
    @param  a   the first vector to cross against.
    @param  b   the second vector to cross against.
    @return     cross product of a and b. */
    static vec3 cross(const vec3& a, const vec3& b) {
        return vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                     a.x * b.y - a.y * b.x };
    }
    /** Calculate the dot product of this vector.
    @param  b   the other vector to dot against.
    @return     dot product of this and the supplied vector. */
    float dot(const vec3& b) const { return dot(*this, b); }
    /** Calculate the dot product the supplied vectors.
    @param  a   the first vector to dot against.
    @param  b   the second vector to dot against.
    @return     dot product of a and b. */
    static float dot(const vec3& a, const vec3& b) {
        return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
    }
};

/** A 4 by 4 matrix of floats. */
struct mat4 {
    // Attributes
    float data[4][4]{
        { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 }
    };

    // Methods
    /** Create a transform matrix looking at a point a given another point.
    @param  eye     the eye position.
    @param  center  the center of the target to look at.
    @aram   up      the up direction.
    @return         a view matrix looking at center from eye. */
    static mat4 lookAt(const vec3& eye, const vec3& center, const vec3& up) {
        const auto f = (center - eye).normalize();
        auto u = up.normalize();
        const auto s = (f.cross(u)).normalize();
        u = s.cross(f);

        mat4 Result;
        Result.data[0][0] = s.x;
        Result.data[1][0] = s.y;
        Result.data[2][0] = s.z;
        Result.data[3][0] = -s.dot(eye);
        Result.data[0][1] = u.x;
        Result.data[1][1] = u.y;
        Result.data[2][1] = u.z;
        Result.data[3][1] = -u.dot(eye);
        Result.data[0][2] = -f.x;
        Result.data[1][2] = -f.y;
        Result.data[2][2] = -f.z;
        Result.data[3][2] = f.dot(eye);
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
        float const& zFar) {
        float const rad = fovY;
        float tanHalfFovy = tanf(rad / float(2));

        mat4 Result;
        Result.data[0][0] = 1.0F / (aspect * tanHalfFovy);
        Result.data[1][1] = 1.0F / (tanHalfFovy);
        Result.data[2][2] = -(zFar + zNear) / (zFar - zNear);
        Result.data[2][3] = -1.0F;
        Result.data[3][2] = -(2.0F * zFar * zNear) / (zFar - zNear);
        return Result;
    }
};

#endif // GEOMETRY_HPP
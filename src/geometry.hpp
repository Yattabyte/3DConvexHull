#pragma once
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP
#include <cmath>

/***/
struct vec3 {
    // Attributes
    float x = 0.0f, y = 0.0f, z = 0.0f;

    // (De)Constructors
    /***/
    ~vec3() = default;
    /***/
    vec3() = default;
    /***/
    vec3(const float& xx, const float& yy, const float& zz)
        : x(xx), y(yy), z(zz) {}
    /***/
    vec3(const vec3& o) = default;
    /***/
    vec3(vec3&& o) noexcept = default;

    // Operators
    /***/
    vec3& operator=(const vec3& p) = default;
    /***/
    vec3& operator=(vec3&& p) noexcept = default;
    /***/
    vec3 operator-(const vec3& o) const {
        return vec3{ x - o.x, y - o.y, z - o.z };
    }
    /***/
    vec3 operator+(const vec3& o) const {
        return vec3{ x + o.x, y + o.y, z + o.z };
    }
    /***/
    bool operator<(const vec3& other) const {
        if (z == other.z) {
            if (x == other.x) {
                return y < other.y;
            }
            return x < other.x;
        }
        return z < other.z;
    };

    // Methods
    /***/
    vec3 normalize() const { return normalize(*this); }
    /***/
    static vec3 normalize(const vec3& v) {
        float length_of_v = sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
        return vec3{ v.x / length_of_v, v.y / length_of_v, v.z / length_of_v };
    }
    /***/
    vec3 cross(const vec3& b) const { return cross(*this, b); }
    /***/
    static vec3 cross(const vec3& a, const vec3& b) {
        return vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                     a.x * b.y - a.y * b.x };
    }
    /***/
    float dot(const vec3& b) const { return dot(*this, b); }
    /***/
    static float dot(const vec3& a, const vec3& b) {
        return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
    }
};

/***/
struct mat4 {
    // Attributes
    float data[4][4]{
        { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 }
    };

    // Methods
    /***/
    static mat4 lookAt(vec3 const& eye, vec3 const& center, vec3 const& up) {
        vec3 f = (center - eye).normalize();
        vec3 u = up.normalize();
        vec3 s = (f.cross(u)).normalize();
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
    /***/
    static mat4 perspective(
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
};

#endif // !GEOMETRY_HPP
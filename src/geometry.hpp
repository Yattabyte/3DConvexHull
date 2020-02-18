#pragma once
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

struct vec3 {
    // Attributes
    float x = 0.0f, y = 0.0f, z = 0.0f;

    // (De)Constructors
    ~vec3() = default;
    vec3() = default;
    vec3(const float& xx, const float& yy, const float& zz)
        : x(xx), y(yy), z(zz) {}
    vec3(const vec3& o) = default;
    vec3(vec3&& o) noexcept = default;

    // Operators
    vec3& operator=(const vec3& p) = default;
    vec3& operator=(vec3&& p) noexcept = default;
    vec3 operator-(const vec3& o) const {
        return vec3{ x - o.x, y - o.y, z - o.z };
    }
    vec3 operator+(const vec3& o) const {
        return vec3{ x + o.x, y + o.y, z + o.z };
    }
};

struct mat4 {
    // Attributes
    float data[4][4]{
        { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 }
    };
};

#endif // !GEOMETRY_HPP
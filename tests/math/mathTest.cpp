#include "mat.hpp"
#include "vec.hpp"
#include <cassert>
#include <iostream>
#include <string>

void vec3Test() noexcept;
void vec4Test() noexcept;
void mat4Test() noexcept;

int main() {
    vec3Test();
    vec4Test();
    mat4Test();
    exit(0);
}

void vec3Test() noexcept {
    // Ensure the single float constructor works as intended
    vec3 vector(1.0F);
    assert(vector.x() == vector.y() && vector.y() == vector.z());
    assert(vector == vec3(1.0F, 1.0F, 1.0F));
    assert(vector != vec3(0.0F, 1.0F, 2.0F));

    // Ensure basic math operations work
    assert(vec3(1.0F) + vec3(1.0F) == vec3(2.0F));
    assert(vec3(1.0F) - vec3(1.0F) == vec3(0.0F));
    assert(vec3(2.0F) / vec3(2.0F) == vec3(1.0F));
    assert(vec3(2.0F) * vec3(2.0F) == vec3(4.0F));

    // Ensure access methods work
    assert(&vector.x() == vector.data());

    // Ensure more complex math methods work
    vector = { 1.0F, 2.0F, 3.0F };
    assert(
        vector.normalize() == vec3(0.267261237F, 0.534522474F, 0.801783681F));
    assert(vector.cross(vec3(3.0F, 2.0F, 1.0F)) == vec3(-4.0F, 8.0F, -4.0F));
    assert(vector.dot(-vector) == -14.0F);
}

void vec4Test() noexcept {
    // Ensure the single float constructor works as intended
    vec4 vector(1.0F);
    assert(
        vector.x() == vector.y() && vector.y() == vector.z() &&
        vector.z() == vector.w());
    assert(vector == vec4(1.0F, 1.0F, 1.0F, 1.0F));
    assert(vector != vec4(0.0F, 1.0F, 2.0F, 3.0F));

    // Ensure basic math operations work
    assert(vec4(1.0F) + vec4(1.0F) == vec4(2.0F));
    assert(vec4(1.0F) - vec4(1.0F) == vec4(0.0F));
    assert(vec4(2.0F) / vec4(2.0F) == vec4(1.0F));
    assert(vec4(2.0F) * vec4(2.0F) == vec4(4.0F));

    // Ensure access methods work
    assert(&vector.x() == vector.data());

    // Ensure more complex math methods work
    vector = { 1.0F, 2.0F, 3.0F, 4.0F };
    assert(
        vector.normalize() ==
        vec4(0.182574183F, 0.365148365F, 0.547722518F, 0.730296731F));
}

void mat4Test() noexcept {
    // Ensure default matrix is an identity matrix
    mat4 matrix;
    assert(
        matrix == mat4(
                      vec4(1, 0, 0, 0), vec4(0, 1, 0, 0), vec4(0, 0, 1, 0),
                      vec4(0, 0, 0, 1)));

    // Ensure negative comparison operator works
    assert(matrix != mat4(vec4(1.0F), vec4(1.0F), vec4(1.0F), vec4(1.0F)));

    // Ensure vec4 array is packed tightly
    assert(matrix.data() == &matrix[0].x());
    assert(&matrix[0].x() + 15ULL == &matrix[3].w());

    const auto pMatrix = mat4::perspective(1.5708F, 1.0F, 0.01F, 10.0F);
    const auto vMatrix =
        mat4::lookAt(vec3(0, 0, -10), vec3(0, 0, 0), vec3(0, 1, 0));
    const auto mMatrix = mat4();
    // const auto MVP = pMatrix * vMatrix * mMatrix;
}
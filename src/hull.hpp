#ifndef HULL_HPP
#define HULL_HPP

#include "geometry.hpp"
#include <vector>

namespace Hull {
struct Triangle {
    // Attributes
    int id, keep;
    int a, b, c;
    int ab, bc, ac;   // adjacent edges index to neighbouring triangle.
    float er, ec, ez; // visible normal to triangular facet.

    // (De)Constructors
    ~Triangle() = default;
    Triangle() = default;
    Triangle(const int& x, const int& y, const int& q)
        : id(0), keep(1), a(x), b(y), c(q), ab(-1), bc(-1), ac(-1), er(0),
          ec(0), ez(0){};
    Triangle(const Triangle& p) = default;
    Triangle(Triangle&& p) noexcept = default;

    // Operators
    Triangle& operator=(const Triangle& p) = default;
    Triangle& operator=(Triangle&& p) noexcept = default;
    bool operator==(const Triangle& t) const {
        return (
            (a == t.a || a == t.b || a == t.c) &&
            (b == t.a || b == t.b || b == t.c) &&
            (c == t.a || c == t.b || c == t.c));
    }

    // Methods
    bool FaceSameDirection(const Triangle& other) const {
        return (er == other.er && ec == other.ec && ez == other.ez);
    }
    bool AreAdjacent(const Triangle& other) const {
        return (
            a == other.ab || a == other.bc || a == other.ac || b == other.ab ||
            b == other.bc || b == other.ac || c == other.ab || c == other.bc ||
            c == other.ac);
    }
    vec3 normal() const { return vec3(er, ec, ez); }
};
struct Point3 {
    // Attributes
    vec3 xyz;

    // (De)Constructors
    ~Point3() = default;
    Point3() = default;
    Point3(const float& a, const float& b, const float& c) : xyz(a, b, c){};
    Point3(const vec3& abc) : xyz(abc){};
    Point3(const Point3& p) = default;
    Point3(Point3&& p) noexcept = default;

    // Operators
    Point3& operator=(const Point3& p) = default;
    Point3& operator=(Point3&& p) noexcept = default;
    bool operator<(const Point3& other) const {
        if (xyz.z == other.xyz.z) {
            if (xyz.x == other.xyz.x) {
                return xyz.y < other.xyz.y;
            }
            return xyz.x < other.xyz.x;
        }
        return xyz.z < other.xyz.z;
    };

    // Methods
    float x() const { return xyz.x; };
    float y() const { return xyz.y; };
    float z() const { return xyz.z; };
};
struct Snork {
    // Attributes
    int id = -1;
    int a = 0, b = 0;

    // (De)Constructors
    ~Snork() = default;
    Snork() = default;
    Snork(const int& i, const int& r, const int& x) : id(i), a(r), b(x){};
    Snork(const Snork& p) = default;
    Snork(Snork&& p) noexcept = default;

    // Operators
    Snork& operator=(const Snork& p) = default;
    Snork& operator=(Snork&& p) noexcept = default;
    bool operator<(const Snork& other) const {
        if (a == other.a)
            return b < other.b;
        return a < other.a;
    };
};

int Generate_3D(std::vector<Point3>& points, std::vector<Triangle>& hull);

int init_hull3D(std::vector<Point3>& pts, std::vector<Triangle>& hull);
void add_coplanar(
    std::vector<Point3>& pts, std::vector<Triangle>& hull, int id);
int cross_test(
    std::vector<Point3>& pts, const int& A, const int& B, const int& C,
    const int& X, float& er, float& ec, float& ez);
} // namespace Hull
#endif // HULL_HPP
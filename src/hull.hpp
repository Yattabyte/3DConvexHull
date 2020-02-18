#ifndef HULL_HPP
#define HULL_HPP

#include "geometry.hpp"
#include <vector>

namespace Hull {
/***/
struct Triangle {
    // Attributes
    int id = 0, keep = 1;
    int a = 0, b = 0, c = 0;
    int ab = -1, bc = -1,
        ac = -1; // adjacent edges index to neighbouring triangle.
    float er = 0.0f, ec = 0.0f,
          ez = 0.0f; // visible normal to triangular facet.

    // (De)Constructors
    /***/
    ~Triangle() = default;
    /***/
    Triangle() = default;
    /***/
    Triangle(const int& x, const int& y, const int& q) : a(x), b(y), c(q){};
    /***/
    Triangle(const Triangle& p) = default;
    /***/
    Triangle(Triangle&& p) noexcept = default;

    // Operators
    /***/
    Triangle& operator=(const Triangle& p) = default;
    /***/
    Triangle& operator=(Triangle&& p) noexcept = default;
    /***/
    bool operator==(const Triangle& t) const {
        return (
            (a == t.a || a == t.b || a == t.c) &&
            (b == t.a || b == t.b || b == t.c) &&
            (c == t.a || c == t.b || c == t.c));
    }

    // Methods
    /***/
    bool FaceSameDirection(const Triangle& other) const {
        return (er == other.er && ec == other.ec && ez == other.ez);
    }
    /***/
    bool AreAdjacent(const Triangle& other) const {
        return (
            a == other.ab || a == other.bc || a == other.ac || b == other.ab ||
            b == other.bc || b == other.ac || c == other.ab || c == other.bc ||
            c == other.ac);
    }
    /***/
    vec3 normal() const { return vec3(er, ec, ez); }
};
/***/
struct Snork {
    // Attributes
    int id = -1;
    int a = 0, b = 0;

    // (De)Constructors
    /***/
    ~Snork() = default;
    /***/
    Snork() = default;
    /***/
    Snork(const int& i, const int& r, const int& x) : id(i), a(r), b(x){};
    /***/
    Snork(const Snork& p) = default;
    /***/
    Snork(Snork&& p) noexcept = default;

    // Operators
    /***/
    Snork& operator=(const Snork& p) = default;
    /***/
    Snork& operator=(Snork&& p) noexcept = default;
    /***/
    bool operator<(const Snork& other) const {
        if (a == other.a)
            return b < other.b;
        return a < other.a;
    };
};

/***/
std::vector<vec3> generate_point_cloud(const float& scale, const size_t& count);
/***/
std::vector<vec3> generate_convex_hull(const std::vector<vec3>& points);
/***/
int init_hull3D(std::vector<vec3>& pts, std::vector<Triangle>& hull);
/***/
void add_coplanar(std::vector<vec3>& pts, std::vector<Triangle>& hull, int id);
/***/
int cross_test(
    std::vector<vec3>& pts, const int& A, const int& B, const int& C,
    const int& X, float& er, float& ec, float& ez);
};     // namespace Hull
#endif // HULL_HPP
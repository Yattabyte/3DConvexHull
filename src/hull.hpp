#ifndef HULL_HPP
#define HULL_HPP

#include "geometry.hpp"
#include <vector>

/** Namespace encapsulating all hull-related operations. */
namespace Hull {
/** A triangle referencing specific point indexes. */
struct Triangle {
    // Attributes
    int id = 0, keep = 1;
    int a = 0, b = 0, c = 0;
    int ab = -1, bc = -1,
        ac = -1; // adjacent edges index to neighbouring triangle.
    float er = 0.0f, ec = 0.0f,
          ez = 0.0f; // visible normal to triangular facet.

    // (De)Constructors
    /** Default destruct this triangle. */
    ~Triangle() = default;
    /** Default construct a triangle. */
    Triangle() = default;
    /** Construct a triangle with specific indices.
    @param  _a  the first point index.
    @param  _b  the second point index.
    @param  _c  the third point index.*/
    Triangle(const int& _a, const int& _b, const int& _c)
        : a(_a), b(_b), c(_c){};
    /** Default copy a triangle. */
    Triangle(const Triangle& p) = default;
    /** Default move a triangle. */
    Triangle(Triangle&& p) noexcept = default;

    // Operators
    /** Default copy-assignment operator. */
    Triangle& operator=(const Triangle& p) = default;
    /** Default move-assignment operator. */
    Triangle& operator=(Triangle&& p) noexcept = default;
    /** Compare whether or not this triangle equals another.
    @param  t   the other triangle to compare against. */
    bool operator==(const Triangle& t) const {
        return (
            (a == t.a || a == t.b || a == t.c) &&
            (b == t.a || b == t.b || b == t.c) &&
            (c == t.a || c == t.b || c == t.c));
    }

    // Methods
    /** Check if this triangle faces the same direction as another.
    @param  other   the other triangle to compare against.
    @return true if this triangle faces the same direction, false otherwise. */
    bool FaceSameDirection(const Triangle& other) const {
        return (er == other.er && ec == other.ec && ez == other.ez);
    }
    /** Checks if this triangle is adjacent to another.
    @param  other   the other triangle to compare against.
    @return true if this triangle is adjacent to another, false otherwise. */
    bool AreAdjacent(const Triangle& other) const {
        return (
            a == other.ab || a == other.bc || a == other.ac || b == other.ab ||
            b == other.bc || b == other.ac || c == other.ab || c == other.bc ||
            c == other.ac);
    }
    /** Compute the normal for this triangle.
    @return the normal vector for this triangle. */
    vec3 normal() const { return vec3(er, ec, ez); }
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

/** Generate a point cloud with a specific number of points and scale.
@param  scale   the scale to use.
@param  count   the number of points to make.
@return a vector of points. */
std::vector<vec3> generate_point_cloud(const float& scale, const size_t& count);
/** Generate a convex hull given a set of points.
@param  points  the points to generate a hull from.
@return a convex hull containing the input points. */
std::vector<vec3> generate_convex_hull(const std::vector<vec3>& points);
};     // namespace Hull
#endif // HULL_HPP
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

    // Operators
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

    // Operators
    bool operator<(const Snork& other) const {
        if (a == other.a)
            return b < other.b;
        return a < other.a;
    };
};

/** Generate a point cloud with a specific number of points and scale.
@param  scale   the scale to use.
@param  count   the number of points to make.
@param  seed    specific seed to use for random generation.
@return a vector of points. */
std::vector<vec3> generate_point_cloud(
    const float& scale, const size_t& count, const unsigned int& seed);
/** Generate a convex hull given a set of points.
@param  points  the points to generate a hull from.
@return a convex hull containing the input points. */
std::vector<vec3> generate_convex_hull(const std::vector<vec3>& points);
};     // namespace Hull
#endif // HULL_HPP
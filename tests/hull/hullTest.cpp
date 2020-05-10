#include "hull.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <string>

//////////////////////////////////////////////////////////////////////
/// Use the shared mini namespace
using namespace mini;

// Constant variables for this test
constexpr auto scale(10.0F);
constexpr auto pointCount(16384ULL);
constexpr auto seed(1234567890U);

void cloudTest(const std::vector<vec3>& pointCloud);
void hullTest(const std::vector<vec3>& pointCloud);

int main() {
    // Generate a point cloud given the above variables
    std::cout << "Generating point cloud given:\n"
              << "\t-scale: " << std::to_string(scale) << "\n"
              << "\t-count: " << std::to_string(pointCount) << "\n"
              << "\t-seed: " << std::to_string(seed) << std::endl;
    const auto pointCloud(Hull::generate_point_cloud(scale, pointCount, seed));

    // Test the point cloud for accuracy
    cloudTest(pointCloud);

    // Test the convex hull for accuracy
    hullTest(pointCloud);

    exit(0);
}

void cloudTest(const std::vector<vec3>& pointCloud) {
    // Ensure number of points match
    assert(pointCloud.size() == pointCount);

    // Ensure scale is accurate
    vec3 max(std::numeric_limits<float>::min());
    vec3 min(std::numeric_limits<float>::max());
    for (const auto& point : pointCloud) {
        if (max.x() < point.x())
            max.x() = point.x();
        if (max.y() < point.y())
            max.y() = point.y();
        if (max.z() < point.z())
            max.z() = point.z();
        if (min.x() > point.x())
            min.x() = point.x();
        if (min.y() > point.y())
            min.y() = point.y();
        if (min.z() > point.z())
            min.z() = point.z();
    }
    [[maybe_unused]] const auto delta = max - min;
    assert(ceilf((delta.x() + delta.y() + delta.z()) / 6.0F) == ceilf(scale));

    // Ensure deterministic point cloud
    assert(
        pointCloud[0].x() == 2.37589645F && pointCloud[0].y() == -3.18982124F &&
        pointCloud[0].z() == 1.83247185F);
}

void hullTest(const std::vector<vec3>& pointCloud) {
    // Attempt to generate a convex hull
    [[maybe_unused]] const auto convexHull(
        Hull::generate_convex_hull(pointCloud));

    // Ensure we have actually have a hull
    assert(!convexHull.empty());
}
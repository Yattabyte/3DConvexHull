#include "hull.hpp"
#include <algorithm>
#include <cstring>
#include <numeric>
#include <random>
#include <tuple>

// Forward Declarations
std::vector<Hull::Triangle> init_hull3D(const std::vector<vec3>& pts);
void add_coplanar(
    const std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull,
    const int& pointID);
std::tuple<float, float, float, float> cross_test(
    const std::vector<vec3>& pts, const int& triA, const int& triB,
    const int& triC, const int& pointID) noexcept;

std::vector<vec3> Hull::generate_point_cloud(
    const float& scale, const size_t& count, const unsigned int& seed) {
    std::uniform_real_distribution<float> randomFloats(-scale, scale);
    std::default_random_engine generator(seed);
    std::vector<vec3> points(count);
    std::generate(std::begin(points), std::end(points), [&]() {
        return vec3{ randomFloats(generator), randomFloats(generator),
                     randomFloats(generator) };
    });
    return points;
}

std::vector<vec3>
Hull::generate_convex_hull(const std::vector<vec3>& unsortedPoints) {
    // Return early if not at-least a tetrahedron
    if (unsortedPoints.size() < 4)
        return {};

    // Sort points
    auto points = unsortedPoints;
    std::sort(points.begin(), points.end());

    // Return early if cannot create hull
    auto tempHull = init_hull3D(points);
    if (tempHull.empty())
        return {};

    // Pick out the hull triangles and renumber.
    const auto hullSize = tempHull.size();
    std::vector<int> taken(hullSize, -1);

    // Create an index from old triangle-id to new triangle-id.
    std::vector<vec3> vertices;
    vertices.reserve(hullSize * 3ULL);
    int count(0);
    for (size_t t = 0ULL; t < hullSize; ++t)
        if (tempHull[t].keep > 0)
            taken[t] = count++;
    for (size_t t = 0ULL; t < hullSize; ++t) {
        auto& temp = tempHull[t];
        if (temp.keep > 0) {
            if (taken[temp.ab] < 0 || taken[temp.bc] < 0 || taken[temp.ac] < 0)
                return {};

            temp.id = taken[t];
            temp.ab = taken[temp.ab];
            temp.bc = taken[temp.bc];
            temp.ac = taken[temp.ac];

            vertices.emplace_back(points[temp.a]);
            vertices.emplace_back(points[temp.b]);
            vertices.emplace_back(points[temp.c]);
        }
    }

    // Ensure all normals point outwards
    const auto center =
        std::accumulate(vertices.cbegin(), vertices.cend(), vec3(0.0F)) /
        vec3(static_cast<float>(vertices.size()));
    for (size_t i = 0ULL; i < vertices.size() - 3ULL; i += 3ULL) {
        const auto& v0 = vertices[i];
        const auto& v1 = vertices[i + 1];
        const auto& v2 = vertices[i + 2];
        const auto normalDirection =
            vec3::normalize(vec3::cross(v1 - v0, v2 - v0));
        const auto triCenter = (v0 + v1 + v2) / vec3(3.0F);
        const auto centerDirection = vec3::normalize(triCenter - center);

        // Compare angle between both directions
        const auto angle = normalDirection.dot(centerDirection);

        // Swap vertices if normal direction points closer towards center
        if (angle < 0.0F)
            std::swap(vertices[i + 1], vertices[i + 2]);
    }

    return vertices;
}

// Initialize the hull to the point where there is a non-zero volume hull.
std::vector<Hull::Triangle> init_hull3D(const std::vector<vec3>& pts) {
    // Check for co-linearity
    const auto& point0(pts[0]);
    const auto& point1(pts[1]);
    const auto& point2(pts[2]);
    const auto test1 = point1 - point0;
    const auto test2 = point2 - point0;
    const auto cross = test1.cross(test2);

    // Avoid adding facets
    if (cross.x() == 0.0F && cross.y() == 0.0F && cross.z() == 0.0F)
        return {};

    // Adjacent facet id number
    std::vector<Hull::Triangle> hull;
    hull.reserve(pts.size() * 4ULL);
    hull.emplace_back(Hull::Triangle{ 0, 1, 0, 1, 2, 1, 1, 1, cross.x(),
                                      cross.y(), cross.z() });
    hull.emplace_back(Hull::Triangle{ 1, 1, 0, 1, 2, 0, 0, 0, -cross.x(),
                                      -cross.y(), -cross.z() });

    // Add points until a non coplanar set of points is achieved.
    std::vector<int> xList;
    auto pointSum(point0 + point1 + point2);
    const auto maxPts(static_cast<int>(pts.size()));
    for (int pointID = 3; pointID < maxPts; ++pointID) {
        const auto& point(pts[pointID]);
        pointSum = pointSum + point;
        const auto middle(pointSum / vec3(1.0F + static_cast<float>(pointID)));
        xList.clear();
        int hvis(-1);

        // Find the first visible plane.
        for (int hullID = static_cast<int>(hull.size() - 1ULL); hullID >= 0;
             --hullID) {
            auto& triangle = hull[hullID];
            const auto delta = point - pts[triangle.a];
            if ((delta.x() * triangle.er) + (delta.y() * triangle.ec) +
                    (delta.z() * triangle.ez) >
                0.0F) {
                hvis = hullID;
                triangle.keep = 0;
                xList.emplace_back(hvis);
                break;
            }
        }
        if (hvis < 0)
            add_coplanar(pts, hull, pointID);
        else if (hvis >= 0) {
            // New triangular facets formed from neighbouring invisible planes
            const auto hullSizeStart = static_cast<int>(hull.size());
            auto numx = static_cast<int>(xList.size());
            const auto facet_adjacent = [&hull, &pts, &point, &xList, &pointID,
                                         &numx, &middle](
                                            const auto& hullID,
                                            const auto& triX, const auto& triY,
                                            auto& triXY) {
                // Point on next triangle
                auto delta(point - pts[triXY.a]);
                if ((delta.x() * triXY.er) + (delta.y() * triXY.ec) +
                        (delta.z() * triXY.ez) >
                    0.0F) {
                    // Add to list.
                    if (triXY.keep == 1) {
                        triXY.keep = 0;
                        xList.emplace_back(hullID);
                        numx++;
                    }
                } else {
                    // make normal vector.
                    Hull::Triangle Tnew{ static_cast<int>(hull.size()),
                                         2,
                                         pointID,
                                         triX,
                                         triY,
                                         -1,
                                         hullID,
                                         -1 };
                    const auto dTest1 = pts[Tnew.a] - pts[Tnew.b];
                    const auto dTest2 = pts[Tnew.a] - pts[Tnew.c];
                    const auto dCross = dTest1.cross(dTest2);

                    // points from new facet towards 'middle'
                    delta = middle - point;

                    // make it point outwards.
                    if ((delta.x() * dCross.x()) + (delta.y() * dCross.y()) +
                            (delta.z() * dCross.z()) >
                        0.0F) {
                        Tnew.er = -dCross.x();
                        Tnew.ec = -dCross.y();
                        Tnew.ez = -dCross.z();
                    } else {
                        Tnew.er = dCross.x();
                        Tnew.ec = dCross.y();
                        Tnew.ez = dCross.z();
                    }

                    // update the touching triangle
                    if ((triXY.a == triX && triXY.b == triY) ||
                        (triXY.a == triY && triXY.b == triX))
                        triXY.ab = static_cast<int>(hull.size());
                    else if (
                        (triXY.a == triX && triXY.c == triY) ||
                        (triXY.a == triY && triXY.c == triX))
                        triXY.ac = static_cast<int>(hull.size());
                    else if (
                        (triXY.b == triX && triXY.c == triY) ||
                        (triXY.b == triY && triXY.c == triX))
                        triXY.bc = static_cast<int>(hull.size());

                    // spawn a new triangle.
                    hull.emplace_back(Tnew);
                }
            };
            for (int x = 0; x < numx; ++x) {
                const auto hullX(hull[xList[x]]);
                facet_adjacent(hullX.ab, hullX.a, hullX.b, hull[hullX.ab]);
                facet_adjacent(hullX.ac, hullX.a, hullX.c, hull[hullX.ac]);
                facet_adjacent(hullX.bc, hullX.b, hullX.c, hull[hullX.bc]);
            }

            // Patch up the new triangles in hull.
            const auto hullSizeEnd = static_cast<int>(hull.size());
            std::vector<Hull::Snork> norts;
            norts.reserve(hullSizeEnd + 1ULL);
            for (int hullID = hullSizeEnd - 1; hullID >= hullSizeStart;
                 --hullID) {
                if (hull[hullID].keep > 1) {
                    norts.emplace_back(
                        Hull::Snork{ hullID, hull[hullID].b, 1 });
                    norts.emplace_back(
                        Hull::Snork{ hullID, hull[hullID].c, 0 });
                    hull[hullID].keep = 1;
                }
            }
            if (norts.size() < 2ULL)
                continue;

            // Sort and link triangle sides.
            std::sort(norts.begin(), norts.end());
            for (auto nort = norts.cbegin(), nortNext = std::next(nort);
                 nortNext < norts.cend(); ++nort, ++nortNext) {
                if (nort->a == nortNext->a) {
                    if (nort->b == 1)
                        hull[nort->id].ab = nortNext->id;
                    else
                        hull[nort->id].ac = nortNext->id;
                    if (nortNext->b == 1)
                        hull[nortNext->id].ab = nort->id;
                    else
                        hull[nortNext->id].ac = nort->id;
                }
            }
        }
    }
    return hull;
}

// Visible edge facet, create 2 new hull plates.
void test_external_edge(
    const std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull,
    const int& pointID, const int& hullID, const int& triA, const int& triB,
    const int& triC, const Hull::Triangle& hullK, int& hullXY, int& triXY) {
    const auto& [sign, er, ec, ez] = cross_test(pts, triA, triB, triC, pointID);
    if (sign < 0.0F) {
        Hull::Triangle upTriangle{ static_cast<int>(hull.size()),
                                   2,
                                   pointID,
                                   triA,
                                   triB,
                                   -1,
                                   -1,
                                   -1,
                                   er,
                                   ec,
                                   ez };
        Hull::Triangle downTriangle{ static_cast<int>(hull.size() + 1ULL),
                                     2,
                                     pointID,
                                     triA,
                                     triB,
                                     -1,
                                     -1,
                                     -1,
                                     -er,
                                     -ec,
                                     -ez };
        if (hullK.er * er + hullK.ec * ec + hullK.ez * ez > 0) {
            upTriangle.bc = hullID;
            downTriangle.bc = triXY;
            triXY = upTriangle.id;
            hullXY = downTriangle.id;
        } else {
            downTriangle.bc = hullID;
            upTriangle.bc = triXY;
            triXY = downTriangle.id;
            hullXY = upTriangle.id;
        }
        hull.emplace_back(upTriangle);
        hull.emplace_back(downTriangle);
    }
};

bool check_direction(
    const int& idA, const int& idB,
    const std::vector<Hull::Triangle>& hull) noexcept {
    return (hull[idA].er * hull[idB].er) + (hull[idA].ec * hull[idB].ec) +
               (hull[idA].ez * hull[idB].ez) >
           0.0F;
}

// Add a point coplanar to the existing planar hull in 3D
void add_coplanar(
    const std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull,
    const int& pointID) {
    // Find visible edges. from external edges.
    const auto numh = static_cast<int>(hull.size());
    for (int hullID = 0; hullID < numh; ++hullID) {
        // Test AB for visibility from new point
        if (hull[hullID].c == hull[hull[hullID].ab].c)
            test_external_edge(
                pts, hull, pointID, hullID, hull[hullID].a, hull[hullID].b,
                hull[hullID].c, hull[hullID], hull[hull[hullID].ab].ab,
                hull[hullID].ab);
        // Test BC for visibility from new point
        if (hull[hullID].a == hull[hull[hullID].bc].a)
            test_external_edge(
                pts, hull, pointID, hullID, hull[hullID].b, hull[hullID].c,
                hull[hullID].a, hull[hullID], hull[hull[hullID].bc].bc,
                hull[hullID].bc);
        // Test AC for visibility from new point
        if (hull[hullID].b == hull[hull[hullID].ac].b)
            test_external_edge(
                pts, hull, pointID, hullID, hull[hullID].a, hull[hullID].c,
                hull[hullID].b, hull[hullID], hull[hull[hullID].ac].ac,
                hull[hullID].ac);
    }

    // Fix up the non assigned hull adjacencies (correctly).
    std::vector<Hull::Snork> norts;
    norts.reserve(hull.size() + 1ULL);
    for (int hullID = static_cast<int>(hull.size() - 1ULL); hullID >= numh;
         --hullID) {
        if (hull[hullID].keep > 1) {
            norts.emplace_back(Hull::Snork{ hullID, hull[hullID].b, 1 });
            norts.emplace_back(Hull::Snork{ hullID, hull[hullID].c, 0 });
            hull[hullID].keep = 1;
        }
    }
    if (norts.size() < 2)
        return;

    std::sort(norts.begin(), norts.end());
    const auto startSize = static_cast<int>(norts.size());
    norts.emplace_back(Hull::Snork{ -1, -1, -1 });
    norts.emplace_back(Hull::Snork{ -2, -2, -2 });

    for (int nortID = 0; nortID < startSize - 1; ++nortID) {
        if (norts[nortID].a != norts[nortID + 1ULL].a)
            continue;

        // Link triangle sides.
        if (norts[nortID].a != norts[nortID + 2ULL].a) {
            // edge of figure case
            if (norts[nortID].b == 1)
                hull[norts[nortID].id].ab = norts[nortID + 1ULL].id;
            else
                hull[norts[nortID].id].ac = norts[nortID + 1ULL].id;
            if (norts[nortID + 1ULL].b == 1)
                hull[norts[nortID + 1ULL].id].ab = norts[nortID].id;
            else
                hull[norts[nortID + 1ULL].id].ac = norts[nortID].id;
            nortID++;
            continue;
        }
        // Internal figure boundary 4 junction case.
        auto& snork0 = norts[nortID];
        auto& snork1 = norts[nortID + 1ULL];
        auto& snork2 = norts[nortID + 2ULL];
        auto& snork3 = norts[nortID + 3ULL];

        // Check normal directions of id and id1..3
        if (check_direction(snork0.id, snork1.id, hull)) {
            if (check_direction(snork0.id, snork2.id, hull))
                std::swap(snork1.id, snork2.id);
            else if (check_direction(snork0.id, snork2.id, hull))
                std::swap(snork1.id, snork3.id);
        }

        if (norts[nortID].b == 1)
            hull[snork0.id].ab = snork1.id;
        else
            hull[snork0.id].ac = snork1.id;

        if (snork1.b == 1)
            hull[snork1.id].ab = snork0.id;
        else
            hull[snork1.id].ac = snork0.id;

        // Use s2 and s3
        if (snork2.b == 1)
            hull[snork2.id].ab = snork3.id;
        else
            hull[snork2.id].ac = snork3.id;

        if (snork3.b == 1)
            hull[snork3.id].ab = snork2.id;
        else
            hull[snork3.id].ac = snork2.id;
        nortID += 3;
    }
}

// Cross product relative sign test.
std::tuple<float, float, float, float> cross_test(
    const std::vector<vec3>& pts, const int& triA, const int& triB,
    const int& triC, const int& pointID) noexcept {
    const auto& pointA(pts[triA]);
    const auto& pointB(pts[triB]);
    const auto& pointC(pts[triC]);
    const auto& pointX(pts[pointID]);

    const auto test1 = pointB - pointA;
    const auto test2 = pointC - pointA;
    const auto test3 = pointX - pointA;

    const auto cross1 = test1.cross(test2);
    const auto cross2 = test1.cross(test3);

    // Look at sign of (ab x ac).(ab x ax)
    const auto globit = (cross1.x() * cross2.x()) + (cross1.y() * cross2.y()) +
                        (cross1.z() * cross2.z());

    return { globit, cross2.x(), cross2.y(), cross2.z() };
}
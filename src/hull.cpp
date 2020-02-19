#include "hull.hpp"
#include <algorithm>
#include <cstring>
#include <random>

// Forward Declarations
std::vector<Hull::Triangle> init_hull3D(const std::vector<vec3>& pts);
void add_coplanar(
    const std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull,
    const int& id);
int cross_test(
    const std::vector<vec3>& pts, const int& A, const int& B, const int& C,
    const int& X, float& er, float& ec, float& ez);

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
    auto temp_hull = init_hull3D(points);
    if (temp_hull.empty())
        return {};

    // just pick out the hull triangles and renumber.
    const auto hull_size = temp_hull.size();
    std::vector<int> taken(hull_size, -1);

    // create an index from old triangle-id to new triangle-id.
    std::vector<vec3> vertices;
    vertices.reserve(hull_size * 3ULL);
    int cnt = 0;
    for (size_t t = 0; t < hull_size; ++t)
        if (temp_hull[t].keep > 0)
            taken[t] = cnt++;
    for (size_t t = 0; t < hull_size; ++t) {
        if (temp_hull[t].keep > 0) {
            auto T = temp_hull[t];
            if (taken[T.ab] < 0 || taken[T.bc] < 0 || taken[T.ac] < 0)
                return {};

            T.id = taken[t];
            T.ab = taken[T.ab];
            T.bc = taken[T.bc];
            T.ac = taken[T.ac];

            vertices.emplace_back(points[T.a]);
            vertices.emplace_back(points[T.b]);
            vertices.emplace_back(points[T.c]);
        }
    }

    return vertices;
}

// Initialize the hull to the point where there is a non-zero volume hull.
std::vector<Hull::Triangle> init_hull3D(const std::vector<vec3>& pts) {
    const auto& point0(pts[0]);
    const auto& point1(pts[1]);
    const auto& point2(pts[2]);

    // check for co-linearity
    const auto test1 = point1 - point0;
    const auto test2 = point2 - point0;
    const auto e0 = test1.y * test2.z - test2.y * test1.z;
    const auto e1 = -test1.x * test2.z + test2.x * test1.z;
    const auto e2 = test1.x * test2.y - test2.x * test1.y;

    // do not add a facet.
    if (e0 == 0.0F && e1 == 0.0F && e2 == 0.0F)
        return {};

    // adjacent facet id number
    std::vector<Hull::Triangle> hull;
    hull.reserve(pts.size() * 4ULL);
    hull.emplace_back(Hull::Triangle{ 0, 1, 0, 1, 2, 1, 1, 1, e0, e1, e2 });
    hull.emplace_back(Hull::Triangle{ 1, 1, 0, 1, 2, 0, 0, 0, -e0, -e1, -e2 });

    // add points until a non coplanar set of points is achieved.
    std::vector<int> xList;
    auto M = point0 + point1 + point2;
    for (int p = 3, max = static_cast<int>(pts.size()); p < max; ++p) {
        xList.clear();
        const auto& point(pts[p]);
        M = M + point;
        const auto m(M / vec3(1.0F + p));

        // find the first visible plane.
        auto hvis(-1);
        for (int h = (int)hull.size() - 1; h >= 0; --h) {
            const auto& t = hull[h];
            const auto& R1 = pts[t.a].x;
            const auto& C1 = pts[t.a].y;
            const auto& Z1 = pts[t.a].z;
            const auto dr = point.x - R1;
            const auto dc = point.y - C1;
            const auto dz = point.z - Z1;

            if (dr * t.er + dc * t.ec + dz * t.ez > 0) {
                hvis = h;
                hull[h].keep = 0;
                xList.emplace_back(hvis);
                break;
            }
        }
        if (hvis < 0)
            add_coplanar(pts, hull, p);
        else if (hvis >= 0) {
            // new triangular facets formed from neighbouring invisible planes
            const auto hullSizeStart = (int)hull.size();
            auto numx = (int)xList.size();
            const auto facet_adjacent =
                [&hull, &pts, &point, &xList, &p, &numx,
                 &m](const auto& xy, const auto& X, const auto& Y, auto& tXY) {
                    // point on next triangle
                    const auto& R1 = pts[tXY.a].x;
                    const auto& C1 = pts[tXY.a].y;
                    const auto& Z1 = pts[tXY.a].z;
                    auto dr = point.x - R1;
                    auto dc = point.y - C1;
                    auto dz = point.z - Z1;

                    if (dr * tXY.er + dc * tXY.ec + dz * tXY.ez > 0) {
                        // add to list.
                        if (tXY.keep == 1) {
                            tXY.keep = 0;
                            xList.emplace_back(xy);
                            numx++;
                        }
                    } else {
                        // spawn a new triangle.
                        Hull::Triangle Tnew{
                            (int)hull.size(), 2, p, X, Y, -1, xy, -1
                        };

                        // make normal vector.
                        const auto dr1 = pts[Tnew.a].x - pts[Tnew.b].x;
                        const auto dr2 = pts[Tnew.a].x - pts[Tnew.c].x;
                        const auto dc1 = pts[Tnew.a].y - pts[Tnew.b].y;
                        const auto dc2 = pts[Tnew.a].y - pts[Tnew.c].y;
                        const auto dz1 = pts[Tnew.a].z - pts[Tnew.b].z;
                        const auto dz2 = pts[Tnew.a].z - pts[Tnew.c].z;
                        const auto er = dc1 * dz2 - dc2 * dz1;
                        const auto ec = -(dr1 * dz2 - dr2 * dz1);
                        const auto ez = dr1 * dc2 - dr2 * dc1;

                        // points from new facet towards 'm'
                        dr = m.x - point.x;
                        dc = m.y - point.y;
                        dz = m.z - point.z;

                        // make it point outwards.
                        if (dr * er + dc * ec + dz * ez > 0) {
                            Tnew.er = -er;
                            Tnew.ec = -ec;
                            Tnew.ez = -ez;
                        } else {
                            Tnew.er = er;
                            Tnew.ec = ec;
                            Tnew.ez = ez;
                        }

                        // update the touching triangle
                        if ((tXY.a == X && tXY.b == Y) ||
                            (tXY.a == Y && tXY.b == X))
                            tXY.ab = (int)hull.size();
                        else if (
                            (tXY.a == X && tXY.c == Y) ||
                            (tXY.a == Y && tXY.c == X))
                            tXY.ac = (int)hull.size();
                        else if (
                            (tXY.b == X && tXY.c == Y) ||
                            (tXY.b == Y && tXY.c == X))
                            tXY.bc = (int)hull.size();

                        hull.emplace_back(Tnew);
                    }
                };
            for (int x = 0; x < numx; ++x) {
                const auto xid = xList[x];
                facet_adjacent(
                    hull[xid].ab, hull[xid].a, hull[xid].b, hull[hull[xid].ab]);
                facet_adjacent(
                    hull[xid].ac, hull[xid].a, hull[xid].c, hull[hull[xid].ac]);
                facet_adjacent(
                    hull[xid].bc, hull[xid].b, hull[xid].c, hull[hull[xid].bc]);
            }

            // patch up the new triangles in hull.
            const auto hullSizeEnd = (int)hull.size();
            std::vector<Hull::Snork> norts;
            norts.reserve(hullSizeEnd + 1ULL);
            for (int q = hullSizeEnd - 1; q >= hullSizeStart; --q) {
                if (hull[q].keep > 1) {
                    norts.emplace_back(Hull::Snork{ q, hull[q].b, 1 });
                    norts.emplace_back(Hull::Snork{ q, hull[q].c, 0 });
                    hull[q].keep = 1;
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

// add a point coplanar to the existing planar hull in 3D
void add_coplanar(
    const std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull,
    const int& id) {
    // find visible edges. from external edges.
    const auto numh = (int)hull.size();
    for (int k = 0; k < numh; ++k) {
        // visible edge facet, create 2 new hull plates.
        const auto test_external_edge = [&pts, &hull, &id, &k](
                                            const auto& A, const auto& B,
                                            const auto& C, auto& hullK,
                                            auto& hullXY, auto& xy) {
            float er(0.0f);
            float ec(0.0f);
            float ez(0.0f);
            if (cross_test(pts, A, B, C, id, er, ec, ez) < 0) {
                Hull::Triangle up{
                    (int)hull.size(), 2, id, A, B, -1, -1, -1, er, ec, ez
                };
                Hull::Triangle down{
                    (int)hull.size() + 1, 2, id, A, B, -1, -1, -1, -er, -ec, -ez
                };
                if (hullK.er * er + hullK.ec * ec + hullK.ez * ez > 0) {
                    up.bc = k;
                    down.bc = xy;
                    xy = up.id;
                    hullXY = down.id;
                } else {
                    down.bc = k;
                    up.bc = xy;
                    xy = down.id;
                    hullXY = up.id;
                }
                hull.emplace_back(up);
                hull.emplace_back(down);
            }
        };
        // test ab for visibility from new point
        if (hull[k].c == hull[hull[k].ab].c)
            test_external_edge(
                hull[k].a, hull[k].b, hull[k].c, hull[k], hull[hull[k].ab].ab,
                hull[k].ab);
        // test bc for visibility from new point
        if (hull[k].a == hull[hull[k].bc].a)
            test_external_edge(
                hull[k].b, hull[k].c, hull[k].a, hull[k], hull[hull[k].bc].bc,
                hull[k].bc);
        // test ac for visibility from new point
        if (hull[k].b == hull[hull[k].ac].b)
            test_external_edge(
                hull[k].a, hull[k].c, hull[k].b, hull[k], hull[hull[k].ac].ac,
                hull[k].ac);
    }

    // Fix up the non assigned hull adjacencies (correctly).
    std::vector<Hull::Snork> norts;
    norts.reserve(hull.size() + 1ULL);
    for (int q = (int)hull.size() - 1; q >= numh; --q) {
        if (hull[q].keep > 1) {
            norts.emplace_back(Hull::Snork{ q, hull[q].b, 1 });
            norts.emplace_back(Hull::Snork{ q, hull[q].c, 0 });
            hull[q].keep = 1;
        }
    }
    if (norts.size() < 2)
        return;

    std::sort(norts.begin(), norts.end());
    const auto startSize = (int)norts.size();
    norts.emplace_back(Hull::Snork{ -1, -1, -1 });
    norts.emplace_back(Hull::Snork{ -2, -2, -2 });

    for (int s = 0; s < startSize - 1; ++s) {
        if (norts[s].a == norts[s + 1].a) {
            // link triangle sides.
            if (norts[s].a != norts[s + 2].a) { // edge of figure case
                if (norts[s].b == 1)
                    hull[norts[s].id].ab = norts[s + 1].id;
                else
                    hull[norts[s].id].ac = norts[s + 1].id;
                if (norts[s + 1].b == 1)
                    hull[norts[s + 1].id].ab = norts[s].id;
                else
                    hull[norts[s + 1].id].ac = norts[s].id;
                s++;
            } else { // internal figure boundary 4 junction case.
                int s1 = s + 1;
                int s2 = s + 2;
                int s3 = s + 3;
                int id0 = norts[s].id;
                int id1 = norts[s1].id;
                int id2 = norts[s2].id;
                int id3 = norts[s3].id;

                // check normal directions of id and id1..3
                auto result = hull[id0].er * hull[id1].er +
                              hull[id0].ec * hull[id1].ec +
                              hull[id0].ez * hull[id1].ez;
                if (result <= 0.0F) {
                    result = hull[id0].er * hull[id2].er +
                             hull[id0].ec * hull[id2].ec +
                             hull[id0].ez * hull[id2].ez;
                    if (result > 0.0F) {
                        std::swap(id1, id2);
                        std::swap(s1, s2);
                    } else {
                        result = hull[id0].er * hull[id3].er +
                                 hull[id0].ec * hull[id3].ec +
                                 hull[id0].ez * hull[id3].ez;
                        if (result > 0.0F) {
                            std::swap(id1, id3);
                            std::swap(s1, s3);
                        }
                    }
                }

                if (norts[s].b == 1)
                    hull[norts[s].id].ab = norts[s1].id;
                else
                    hull[norts[s].id].ac = norts[s1].id;

                if (norts[s1].b == 1)
                    hull[norts[s1].id].ab = norts[s].id;
                else
                    hull[norts[s1].id].ac = norts[s].id;

                // use s2 and s3
                if (norts[s2].b == 1)
                    hull[norts[s2].id].ab = norts[s3].id;
                else
                    hull[norts[s2].id].ac = norts[s3].id;

                if (norts[s3].b == 1)
                    hull[norts[s3].id].ab = norts[s2].id;
                else
                    hull[norts[s3].id].ac = norts[s2].id;
                s += 3;
            }
        }
    }
}

// cross product relative sign test.
// remembers the cross product of (ab x cx)
int cross_test(
    const std::vector<vec3>& pts, const int& A, const int& B, const int& C,
    const int& X, float& er, float& ec, float& ez) {
    const auto Ar = pts[A].x;
    const auto Ac = pts[A].y;
    const auto Az = pts[A].z;

    const auto Br = pts[B].x;
    const auto Bc = pts[B].y;
    const auto Bz = pts[B].z;

    const auto Cr = pts[C].x;
    const auto Cc = pts[C].y;
    const auto Cz = pts[C].z;

    const auto Xr = pts[X].x;
    const auto Xc = pts[X].y;
    const auto Xz = pts[X].z;

    const auto ABr = Br - Ar;
    const auto ABc = Bc - Ac;
    const auto ABz = Bz - Az;

    const auto ACr = Cr - Ar;
    const auto ACc = Cc - Ac;
    const auto ACz = Cz - Az;

    const auto AXr = Xr - Ar;
    const auto AXc = Xc - Ac;
    const auto AXz = Xz - Az;

    er = (ABc * AXz - ABz * AXc);
    ec = -(ABr * AXz - ABz * AXr);
    ez = (ABr * AXc - ABc * AXr);

    const auto kr = ABc * ACz - ABz * ACc;
    const auto kc = -(ABr * ACz - ABz * ACr);
    const auto kz = ABr * ACc - ABc * ACr;

    // look at sign of (ab x ac).(ab x ax)
    const auto globit = kr * er + kc * ec + kz * ez;
    if (globit > 0)
        return 1;
    if (globit == 0)
        return 0;

    return -1;
}
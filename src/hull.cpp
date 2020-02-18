#include "hull.hpp"
#include <algorithm>
#include <cstring>
#include <random>

// Forward Declarations
int init_hull3D(std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull);
void add_coplanar(
    std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull, int id);
int cross_test(
    std::vector<vec3>& pts, const int& A, const int& B, const int& C,
    const int& X, float& er, float& ec, float& ez);

std::vector<vec3>
Hull::generate_point_cloud(const float& scale, const size_t& count) {
    std::uniform_real_distribution<float> randomFloats(-scale, scale);
    std::default_random_engine generator;
    std::vector<vec3> points(count);
    std::generate(std::begin(points), std::end(points), [&]() {
        return vec3{ randomFloats(generator), randomFloats(generator),
                     randomFloats(generator) };
    });
    return points;
}

// Generate a convex hull from a 3D point set
std::vector<vec3>
Hull::generate_convex_hull(const std::vector<vec3>& unsortedPoints) {
    // Return early if not at-least a tetrahedron
    if (unsortedPoints.size() < 4)
        return {};

    // Sort points
    auto points = unsortedPoints;
    sort(points.begin(), points.end());

    std::vector<Triangle> temp_hull;
    if (init_hull3D(points, temp_hull) != 0)
        return {};

    // just pick out the hull triangles and renumber.
    const auto hull_size = temp_hull.size();
    std::vector<int> taken(hull_size, -1);

    std::vector<vec3> vertices;
    vertices.reserve(hull_size * 3ULL);
    int cnt = 0;
    for (size_t t = 0; t < hull_size;
         ++t) { // create an index from old triangle-id to new triangle-id.
        if (temp_hull[t].keep > 0) { // point index remains unchanged.
            taken[t] = cnt;
            cnt++;
        }
    }

    for (size_t t = 0; t < hull_size;
         ++t) { // create an index from old triangle-id to new triangle-id.
        if (temp_hull[t].keep > 0) { // point index remains unchanged.
            Triangle T = temp_hull[t];
            T.id = taken[t];
            if (taken[T.ab] < 0)
                return {};
            T.ab = taken[T.ab];

            if (taken[T.bc] < 0)
                return {};
            T.bc = taken[T.bc];

            if (taken[T.ac] < 0)
                return {};
            T.ac = taken[T.ac];

            vertices.emplace_back(points[T.a]);
            vertices.emplace_back(points[T.b]);
            vertices.emplace_back(points[T.c]);
        }
    }

    return vertices;
}

// Initialize the hull to the point where there is a not zero volume hull.
int init_hull3D(std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull) {
    int nump = (int)pts.size();
    std::vector<Hull::Snork> norts;
    hull.reserve(nump * 4);

    float mr = 0;
    float mc = 0;
    float mz = 0;
    float Mr = 0;
    float Mc = 0;
    float Mz = 0;

    Hull::Triangle T1(0, 1, 2);
    float r0 = pts[0].x;
    float c0 = pts[0].y;
    float z0 = pts[0].z;
    float r1 = pts[1].x;
    float c1 = pts[1].y;
    float z1 = pts[1].z;
    float r2 = pts[2].x;
    float c2 = pts[2].y;
    float z2 = pts[2].z;

    Mr = r0 + r1 + r2;
    Mc = c0 + c1 + c2;
    Mz = z0 + z1 + z2;

    // check for co-linearity
    float r01 = r1 - r0;
    float r02 = r2 - r0;
    float c01 = c1 - c0;
    float c02 = c2 - c0;
    float z01 = z1 - z0;
    float z02 = z2 - z0;

    float e0 = c01 * z02 - c02 * z01;
    float e1 = -r01 * z02 + r02 * z01;
    float e2 = r01 * c02 - r02 * c01;

    if (e0 == 0 && e1 == 0 && e2 == 0) // do not add a facet.
        return (-1);

    T1.id = 0;
    T1.er = e0;
    T1.ec = e1;
    T1.ez = e2;

    T1.ab = 1; // adjacent facet id number
    T1.ac = 1;
    T1.bc = 1;

    hull.push_back(T1);

    T1.id = 1;
    T1.er = -e0;
    T1.ec = -e1;
    T1.ez = -e2;

    T1.ab = 0;
    T1.ac = 0;
    T1.bc = 0;

    hull.push_back(T1);
    std::vector<int> xlist;
    Hull::Triangle Tnew{};

    for (int p = 3; p < nump;
         p++) { // add points until a non coplanar set of points is achieved.
        vec3& pt = pts[p];

        Mr += pt.x;
        mr = Mr / (p + 1);
        Mc += pt.y;
        mc = Mc / (p + 1);
        Mz += pt.z;
        mz = Mz / (p + 1);

        // find the first visible plane.
        int hvis = -1;
        float r = pt.x;
        float c = pt.y;
        float z = pt.z;
        xlist.clear();

        for (int h = (int)hull.size() - 1; h >= 0; h--) {
            Hull::Triangle& t = hull[h];
            float R1 = pts[t.a].x;
            float C1 = pts[t.a].y;
            float Z1 = pts[t.a].z;

            float dr = r - R1;
            float dc = c - C1;
            float dz = z - Z1;

            float d = dr * t.er + dc * t.ec + dz * t.ez;

            if (d > 0) {
                hvis = h;
                hull[h].keep = 0;
                xlist.push_back(hvis);
                break;
            }
        }
        if (hvis < 0) {
            add_coplanar(pts, hull, p);
        }
        if (hvis >= 0) {
            // new triangular facets are formed from neighbouring invisible
            // planes.
            int numh = (int)hull.size();
            int numx = (int)xlist.size();
            for (int x = 0; x < numx; x++) {
                int xid = xlist[x];
                int ab = hull[xid].ab; // facet adjacent to line ab
                Hull::Triangle& tAB = hull[ab];

                float R1 = pts[tAB.a].x; // point on next triangle
                float C1 = pts[tAB.a].y;
                float Z1 = pts[tAB.a].z;

                float dr = r - R1;
                float dc = c - C1;
                float dz = z - Z1;

                float d = dr * tAB.er + dc * tAB.ec + dz * tAB.ez;

                if (d > 0) { // add to list.
                    if (hull[ab].keep == 1) {
                        hull[ab].keep = 0;
                        xlist.push_back(ab);
                        numx++;
                    }
                } else { // spawn a new triangle.
                    Tnew.id = (int)hull.size();
                    Tnew.keep = 2;
                    Tnew.a = p;
                    Tnew.b = hull[xid].a;
                    Tnew.c = hull[xid].b;

                    Tnew.ab = -1;
                    Tnew.ac = -1;
                    Tnew.bc = ab;

                    // make normal vector.
                    float dr1 = pts[Tnew.a].x - pts[Tnew.b].x;
                    float dr2 = pts[Tnew.a].x - pts[Tnew.c].x;
                    float dc1 = pts[Tnew.a].y - pts[Tnew.b].y;
                    float dc2 = pts[Tnew.a].y - pts[Tnew.c].y;
                    float dz1 = pts[Tnew.a].z - pts[Tnew.b].z;
                    float dz2 = pts[Tnew.a].z - pts[Tnew.c].z;

                    float er = (dc1 * dz2 - dc2 * dz1);
                    float ec = -(dr1 * dz2 - dr2 * dz1);
                    float ez = (dr1 * dc2 - dr2 * dc1);

                    dr = mr - r; // points from new facet towards [mr,mc,mz]
                    dc = mc - c;
                    dz = mz - z;
                    // make it point outwards.

                    float dromadery = dr * er + dc * ec + dz * ez;

                    if (dromadery > 0) {
                        Tnew.er = -er;
                        Tnew.ec = -ec;
                        Tnew.ez = -ez;
                    } else {
                        Tnew.er = er;
                        Tnew.ec = ec;
                        Tnew.ez = ez;
                    }

                    // update the touching triangle tAB
                    int A = hull[xid].a;
                    int B = hull[xid].b;
                    if ((tAB.a == A && tAB.b == B) ||
                        (tAB.a == B && tAB.b == A)) {
                        tAB.ab = (int)hull.size();
                    } else if (
                        (tAB.a == A && tAB.c == B) ||
                        (tAB.a == B && tAB.c == A)) {
                        tAB.ac = (int)hull.size();
                    } else if (
                        (tAB.b == A && tAB.c == B) ||
                        (tAB.b == B && tAB.c == A)) {
                        tAB.bc = (int)hull.size();
                    }

                    hull.push_back(Tnew);
                }

                // second side of the struck out triangle

                int ac = hull[xid].ac; // facet adjacent to line ac
                Hull::Triangle& tAC = hull[ac];

                R1 = pts[tAC.a].x; // point on next triangle
                C1 = pts[tAC.a].y;
                Z1 = pts[tAC.a].z;

                dr = r - R1;
                dc = c - C1;
                dz = z - Z1;

                d = dr * tAC.er + dc * tAC.ec + dz * tAC.ez;

                if (d > 0) { // add to list.
                    if (hull[ac].keep == 1) {
                        hull[ac].keep = 0;
                        xlist.push_back(ac);
                        numx++;
                    }
                } else { // spawn a new triangle.
                    Tnew.id = (int)hull.size();
                    Tnew.keep = 2;
                    Tnew.a = p;
                    Tnew.b = hull[xid].a;
                    Tnew.c = hull[xid].c;

                    Tnew.ab = -1;
                    Tnew.ac = -1;
                    Tnew.bc = ac;

                    // make normal vector.
                    float dr1 = pts[Tnew.a].x - pts[Tnew.b].x;
                    float dr2 = pts[Tnew.a].x - pts[Tnew.c].x;
                    float dc1 = pts[Tnew.a].y - pts[Tnew.b].y;
                    float dc2 = pts[Tnew.a].y - pts[Tnew.c].y;
                    float dz1 = pts[Tnew.a].z - pts[Tnew.b].z;
                    float dz2 = pts[Tnew.a].z - pts[Tnew.c].z;

                    float er = (dc1 * dz2 - dc2 * dz1);
                    float ec = -(dr1 * dz2 - dr2 * dz1);
                    float ez = (dr1 * dc2 - dr2 * dc1);

                    dr = mr - r; // points from new facet towards [mr,mc,mz]
                    dc = mc - c;
                    dz = mz - z;
                    // make it point outwards.

                    float dromadery = dr * er + dc * ec + dz * ez;

                    if (dromadery > 0) {
                        Tnew.er = -er;
                        Tnew.ec = -ec;
                        Tnew.ez = -ez;
                    } else {
                        Tnew.er = er;
                        Tnew.ec = ec;
                        Tnew.ez = ez;
                    }
                    // update the touching triangle tAC
                    int A = hull[xid].a;
                    int C = hull[xid].c;
                    if ((tAC.a == A && tAC.b == C) ||
                        (tAC.a == C && tAC.b == A)) {
                        tAC.ab = (int)hull.size();
                    } else if (
                        (tAC.a == A && tAC.c == C) ||
                        (tAC.a == C && tAC.c == A)) {
                        tAC.ac = (int)hull.size();
                    } else if (
                        (tAC.b == A && tAC.c == C) ||
                        (tAC.b == C && tAC.c == A)) {
                        tAC.bc = (int)hull.size();
                    }

                    hull.push_back(Tnew);
                }

                // third side of the struck out triangle

                int bc = hull[xid].bc; // facet adjacent to line ac
                Hull::Triangle& tBC = hull[bc];

                R1 = pts[tBC.a].x; // point on next triangle
                C1 = pts[tBC.a].y;
                Z1 = pts[tBC.a].z;

                dr = r - R1;
                dc = c - C1;
                dz = z - Z1;

                d = dr * tBC.er + dc * tBC.ec + dz * tBC.ez;

                if (d > 0) { // add to list.
                    if (hull[bc].keep == 1) {
                        hull[bc].keep = 0;
                        xlist.push_back(bc);
                        numx++;
                    }
                } else { // spawn a new triangle.
                    Tnew.id = (int)hull.size();
                    Tnew.keep = 2;
                    Tnew.a = p;
                    Tnew.b = hull[xid].b;
                    Tnew.c = hull[xid].c;

                    Tnew.ab = -1;
                    Tnew.ac = -1;
                    Tnew.bc = bc;

                    // make normal vector.
                    float dr1 = pts[Tnew.a].x - pts[Tnew.b].x;
                    float dr2 = pts[Tnew.a].x - pts[Tnew.c].x;
                    float dc1 = pts[Tnew.a].y - pts[Tnew.b].y;
                    float dc2 = pts[Tnew.a].y - pts[Tnew.c].y;
                    float dz1 = pts[Tnew.a].z - pts[Tnew.b].z;
                    float dz2 = pts[Tnew.a].z - pts[Tnew.c].z;

                    float er = (dc1 * dz2 - dc2 * dz1);
                    float ec = -(dr1 * dz2 - dr2 * dz1);
                    float ez = (dr1 * dc2 - dr2 * dc1);

                    dr = mr - r; // points from new facet towards [mr,mc,mz]
                    dc = mc - c;
                    dz = mz - z;
                    // make it point outwards.

                    float dromadery = dr * er + dc * ec + dz * ez;

                    if (dromadery > 0) {
                        Tnew.er = -er;
                        Tnew.ec = -ec;
                        Tnew.ez = -ez;
                    } else {
                        Tnew.er = er;
                        Tnew.ec = ec;
                        Tnew.ez = ez;
                    }

                    // update the touching triangle tBC
                    int B = hull[xid].b;
                    int C = hull[xid].c;
                    if ((tBC.a == B && tBC.b == C) ||
                        (tBC.a == C && tBC.b == B)) {
                        tBC.ab = (int)hull.size();
                    } else if (
                        (tBC.a == B && tBC.c == C) ||
                        (tBC.a == C && tBC.c == B)) {
                        tBC.ac = (int)hull.size();
                    } else if (
                        (tBC.b == B && tBC.c == C) ||
                        (tBC.b == C && tBC.c == B)) {
                        tBC.bc = (int)hull.size();
                    }

                    hull.push_back(Tnew);
                }
            }
            // patch up the new triangles in hull.

            int numN = (int)hull.size();
            // vector<Snork> norts;
            int numS = (int)norts.size();
            int nums = 0;
            Hull::Snork snort;
            for (int q = numN - 1; q >= numh; q--) {
                if (hull[q].keep > 1) {
                    if (nums < numS) {
                        norts[nums].id = q;
                        norts[nums].a = hull[q].b;
                        norts[nums].b = 1;

                        nums++;
                    } else {
                        snort.id = q;
                        snort.a = hull[q].b;
                        snort.b = 1;

                        norts.push_back(snort);
                        nums++;
                        numS = (int)norts.size();
                    }

                    if (nums < numS) {
                        norts[nums].id = q;
                        norts[nums].a = hull[q].c;
                        norts[nums].b = 0;

                        nums++;
                    } else {
                        snort.a = hull[q].c;
                        snort.b = 0;
                        norts.push_back(snort);
                        nums++;
                        numS = (int)norts.size();
                    }

                    hull[q].keep = 1;
                }
            }

            sort(norts.begin(), norts.begin() + nums);

            if (nums >= 2) {
                for (int s = 0; s < nums - 1; s++) {
                    if (norts[s].a == norts[s + 1].a) {
                        // link triangle sides.
                        if (norts[s].b == 1) {
                            hull[norts[s].id].ab = norts[s + 1].id;
                        } else {
                            hull[norts[s].id].ac = norts[s + 1].id;
                        }

                        if (norts[s + 1].b == 1) {
                            hull[norts[s + 1].id].ab = norts[s].id;
                        } else {
                            hull[norts[s + 1].id].ac = norts[s].id;
                        }
                    }
                }
            }
        }
    }
    return (0);
}

// add a point coplanar to the existing planar hull in 3D
void add_coplanar(
    std::vector<vec3>& pts, std::vector<Hull::Triangle>& hull, int id) {
    int numh = (int)hull.size();
    float er;
    float ec;
    float ez;
    for (int k = 0; k < numh; k++) {
        // find visible edges. from external edges.

        if (hull[k].c == hull[hull[k].ab].c) { // ->  ab is an external edge.
                                               // test this edge for visibility
                                               // from new point points[id].
            int A = hull[k].a;
            int B = hull[k].b;
            int C = hull[k].c;

            int zot = cross_test(pts, A, B, C, id, er, ec, ez);

            if (zot < 0) { // visible edge facet, create 2 new hull plates.
                Hull::Triangle up;
                Hull::Triangle down;
                up.keep = 2;
                up.id = (int)hull.size();
                up.a = id;
                up.b = A;
                up.c = B;

                up.er = er;
                up.ec = ec;
                up.ez = ez;
                up.ab = -1;
                up.ac = -1;

                down.keep = 2;
                down.id = (int)hull.size() + 1;
                down.a = id;
                down.b = A;
                down.c = B;

                down.ab = -1;
                down.ac = -1;
                down.er = -er;
                down.ec = -ec;
                down.ez = -ez;

                float xx = hull[k].er * er + hull[k].ec * ec + hull[k].ez * ez;
                if (xx > 0) {
                    up.bc = k;
                    down.bc = hull[k].ab;

                    hull[k].ab = up.id;
                    hull[down.bc].ab = down.id;
                } else {
                    down.bc = k;
                    up.bc = hull[k].ab;

                    hull[k].ab = down.id;
                    hull[up.bc].ab = up.id;
                }

                hull.push_back(up);
                hull.push_back(down);
            }
        }

        if (hull[k].a == hull[hull[k].bc].a) { // bc is an external edge.
                                               // test this edge for visibility
                                               // from new point points[id].
            int A = hull[k].b;
            int B = hull[k].c;
            int C = hull[k].a;

            int zot = cross_test(pts, A, B, C, id, er, ec, ez);

            if (zot < 0) { // visible edge facet, create 2 new hull plates.
                Hull::Triangle up;
                Hull::Triangle down;
                up.keep = 2;
                up.id = (int)hull.size();
                up.a = id;
                up.b = A;
                up.c = B;

                up.er = er;
                up.ec = ec;
                up.ez = ez;
                up.ab = -1;
                up.ac = -1;

                down.keep = 2;
                down.id = (int)hull.size() + 1;
                down.a = id;
                down.b = A;
                down.c = B;

                down.ab = -1;
                down.ac = -1;
                down.er = -er;
                down.ec = -ec;
                down.ez = -ez;

                float xx = hull[k].er * er + hull[k].ec * ec + hull[k].ez * ez;
                if (xx > 0) {
                    up.bc = k;
                    down.bc = hull[k].bc;

                    hull[k].bc = up.id;
                    hull[down.bc].bc = down.id;
                } else {
                    down.bc = k;
                    up.bc = hull[k].bc;

                    hull[k].bc = down.id;
                    hull[up.bc].bc = up.id;
                }

                hull.push_back(up);
                hull.push_back(down);
            }
        }

        if (hull[k].b == hull[hull[k].ac].b) { // ac is an external edge.
                                               // test this edge for visibility
                                               // from new point points[id].
            int A = hull[k].a;
            int B = hull[k].c;
            int C = hull[k].b;

            int zot = cross_test(pts, A, B, C, id, er, ec, ez);

            if (zot < 0) { // visible edge facet, create 2 new hull plates.
                Hull::Triangle up;
                Hull::Triangle down;
                up.keep = 2;
                up.id = (int)hull.size();
                up.a = id;
                up.b = A;
                up.c = B;

                up.er = er;
                up.ec = ec;
                up.ez = ez;
                up.ab = -1;
                up.ac = -1;

                down.keep = 2;
                down.id = (int)hull.size() + 1;
                down.a = id;
                down.b = A;
                down.c = B;

                down.ab = -1;
                down.ac = -1;
                down.er = -er;
                down.ec = -ec;
                down.ez = -ez;

                float xx = hull[k].er * er + hull[k].ec * ec + hull[k].ez * ez;
                if (xx > 0) {
                    up.bc = k;
                    down.bc = hull[k].ac;

                    hull[k].ac = up.id;
                    hull[down.bc].ac = down.id;
                } else {
                    down.bc = k;
                    up.bc = hull[k].ac;

                    hull[k].ac = down.id;
                    hull[up.bc].ac = up.id;
                }

                hull.push_back(up);
                hull.push_back(down);
            }
        }
    }

    // Fix up the non assigned hull adjacencies (correctly).
    int numN = (int)hull.size();
    std::vector<Hull::Snork> norts;
    Hull::Snork snort;
    for (int q = numN - 1; q >= numh; q--) {
        if (hull[q].keep > 1) {
            snort.id = q;
            snort.a = hull[q].b;
            snort.b = 1;
            norts.push_back(snort);

            snort.a = hull[q].c;
            snort.b = 0;
            norts.push_back(snort);

            hull[q].keep = 1;
        }
    }

    sort(norts.begin(), norts.end());
    int nums = (int)norts.size();
    Hull::Snork snor;
    snor.id = -1;
    snor.a = -1;
    snor.b = -1;
    norts.push_back(snor);
    snor.id = -2;
    snor.a = -2;
    snor.b = -2;
    norts.push_back(snor);

    if (nums >= 2) {
        for (int s = 0; s < nums - 1; s++) {
            if (norts[s].a == norts[s + 1].a) {
                // link triangle sides.
                if (norts[s].a != norts[s + 2].a) { // edge of figure case
                    if (norts[s].b == 1) {
                        hull[norts[s].id].ab = norts[s + 1].id;
                    } else {
                        hull[norts[s].id].ac = norts[s + 1].id;
                    }

                    if (norts[s + 1].b == 1) {
                        hull[norts[s + 1].id].ab = norts[s].id;
                    } else {
                        hull[norts[s + 1].id].ac = norts[s].id;
                    }
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
                    float barf = hull[id0].er * hull[id1].er +
                                 hull[id0].ec * hull[id1].ec +
                                 hull[id0].ez * hull[id1].ez;
                    if (barf > 0) {
                    } else {
                        barf = hull[id0].er * hull[id2].er +
                               hull[id0].ec * hull[id2].ec +
                               hull[id0].ez * hull[id2].ez;
                        if (barf > 0) {
                            int tmp = id2;
                            id2 = id1;
                            id1 = tmp;
                            tmp = s2;
                            s2 = s1;
                            s1 = tmp;
                        } else {
                            barf = hull[id0].er * hull[id3].er +
                                   hull[id0].ec * hull[id3].ec +
                                   hull[id0].ez * hull[id3].ez;
                            if (barf > 0) {
                                int tmp = id3;
                                id3 = id1;
                                id1 = tmp;
                                tmp = s3;
                                s3 = s1;
                                s1 = tmp;
                            }
                        }
                    }

                    if (norts[s].b == 1) {
                        hull[norts[s].id].ab = norts[s1].id;
                    } else {
                        hull[norts[s].id].ac = norts[s1].id;
                    }

                    if (norts[s1].b == 1) {
                        hull[norts[s1].id].ab = norts[s].id;
                    } else {
                        hull[norts[s1].id].ac = norts[s].id;
                    }

                    // use s2 and s3

                    if (norts[s2].b == 1) {
                        hull[norts[s2].id].ab = norts[s3].id;
                    } else {
                        hull[norts[s2].id].ac = norts[s3].id;
                    }

                    if (norts[s3].b == 1) {
                        hull[norts[s3].id].ab = norts[s2].id;
                    } else {
                        hull[norts[s3].id].ac = norts[s2].id;
                    }

                    s += 3;
                }
            }
        }
    }
}

// cross product relative sign test.
// remembers the cross product of (ab x cx)
int cross_test(
    std::vector<vec3>& pts, const int& A, const int& B, const int& C,
    const int& X, float& er, float& ec, float& ez) {
    float Ar = pts[A].x;
    float Ac = pts[A].y;
    float Az = pts[A].z;

    float Br = pts[B].x;
    float Bc = pts[B].y;
    float Bz = pts[B].z;

    float Cr = pts[C].x;
    float Cc = pts[C].y;
    float Cz = pts[C].z;

    float Xr = pts[X].x;
    float Xc = pts[X].y;
    float Xz = pts[X].z;

    float ABr = Br - Ar;
    float ABc = Bc - Ac;
    float ABz = Bz - Az;

    float ACr = Cr - Ar;
    float ACc = Cc - Ac;
    float ACz = Cz - Az;

    float AXr = Xr - Ar;
    float AXc = Xc - Ac;
    float AXz = Xz - Az;

    er = (ABc * AXz - ABz * AXc);
    ec = -(ABr * AXz - ABz * AXr);
    ez = (ABr * AXc - ABc * AXr);

    float kr = (ABc * ACz - ABz * ACc);
    float kc = -(ABr * ACz - ABz * ACr);
    float kz = (ABr * ACc - ABc * ACr);

    //  look at sign of (ab x ac).(ab x ax)

    float globit = kr * er + kc * ec + kz * ez;

    if (globit > 0)
        return (1);
    if (globit == 0)
        return (0);

    return (-1);
}
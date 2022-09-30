#pragma once

#include <common/matrix_types.h>
#include <tuple>
#include <vector>

namespace sanescan::edgegraph3d {

using ulong = unsigned long;

inline Vec4f to_vec4(const Vec2f& a, const Vec2f& b)
{
    return Vec4f(a.x(), a.y(), b.x(), b.y());
}

// triangulation position, reprojection coords, reprojection ids
struct ReprejectedPoint3dData {
    Vec3f pos;
    std::vector<Vec2f> reprojected_coords;
    std::vector<int> reprojection_ids;
};

struct ReprojectionIds3View {
    int a = 0;
    int b = 0;
    int c = 0;
};

struct Line3D {
    Vec3f start;
    Vec3f end;

    Line3D(const Vec3f& start, const Vec3f& end) : start{start}, end{end} {}
    Vec3f direction() const { return end - start; }
};

struct ExtremePair {
    ulong a = 0;
    ulong b = 0;

    ExtremePair(ulong a, ulong b) : a{a}, b{b} {}
};

struct ExtremeNodeData2D {
    ulong node_id = 0;
    Vec2f coords;
};

struct ExtremeNodeDataWithDirection {
    ulong node_id = 0;
    Vec2f coords;
    Vec2f direction;
};

struct PolylineSearchResult {
    ulong polyline_id = 0;
    ulong closest_segment_id = 0;
    Vec2f projection;
    float distance = 0;
};

} // namespace sanescan::edgegraph3d

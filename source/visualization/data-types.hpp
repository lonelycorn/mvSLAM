#pragma once

#include <cstdint>
#include <unordered_map>

#include <base/math.hpp>
#include <base/space.hpp>

namespace mvSLAM
{
struct VisualizationTypes
{
    using CameraId = uint32_t;

    using PointId = uint32_t;
    using Point3 = Vector3Type;

    using PointCloudId = uint32_t;
    using PointCloud = std::unordered_map<PointId, Point3>;
};

}

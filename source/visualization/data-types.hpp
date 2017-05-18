#pragma once

#include <cstdint>
#include <unordered_map>

#include <base/math.hpp>
#include <base/space.hpp>

namespace mvSLAM
{
struct VisualizationTypes
{
    using CameraId = Id::Type;

    using PointId = Id::Type;
    using Point3 = Vector3Type;

    using PointCloudId = Id::Type;
    using PointCloud = std::unordered_map<PointId, Point3>;
};

}

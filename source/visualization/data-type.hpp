#pragma once

#include <unordered_map>

#include <math/matrix.hpp>
#include <base/data-type.hpp>

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

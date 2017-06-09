#pragma once
#include <math/lie-group.hpp>
#include <math/state-estimate.hpp>
#include <base/image.hpp>
#include <vector>
#include <unordered_map>

namespace mvSLAM
{

/// Unique identifier
class Id
{
public:
    using Type = std::size_t;
    static constexpr Type INVALID = static_cast<Type>(-1);
};

using Transformation = SE3;
using TransformationUncertainty = Matrix6Type;
using TransformationEstimate = StateEstimate<Transformation, TransformationUncertainty>;

using Point3 = Vector3Type;
using Point3Uncertainty = Matrix3Type;
using Point3Estimate = StateEstimate<Point3, Point3Uncertainty>;

using Point2 = Vector2Type;
using Point2Uncertainty = Matrix2Type;
using Point2Estimate = StateEstimate<Point2, Point2Uncertainty>;

using Point3Cloud = std::unordered_map<Id::Type, Point3>;
using Point2Cloud = std::unordered_map<Id::Type, Point2>;

struct KeyPointImage
{
    Id::Type id;
    Point2Cloud key_points;
    ImageGrayscale image;

    KeyPointImage();
};

struct KeyPointImagePair
{
    KeyPointImage base;
    KeyPointImage pair;
    
    KeyPointImagePair();
};

}

#pragma once
#include <math/space.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

/**
 * Helper functions to interface with GTSAM
 */
namespace mvSLAM
{

gtsam::Point2 mvSLAM_Point2_to_gtsam_Point2(const mvSLAM::Point2 &p);
mvSLAM::Point2 gtsam_Point2_to_mvSLAM_Point2(const gtsam::Point2 &p);

gtsam::Point3 mvSLAM_Point3_to_gtsam_Point3(const mvSLAM::Point3 &p);
mvSLAM::Point3 gtsam_Point3_to_mvSLAM_Point3(const gtsam::Point3 &p);

gtsam::Rot3 SO3_to_Rot3(const SO3 &p);
SO3 Rot3_to_SO3(const gtsam::SO3 &p);

gtsam::Pose3 SE3_to_Pose3(const SE3 &p);
SE3 Pose3_to_SE3(const gtsam::Pose3 &p);

}

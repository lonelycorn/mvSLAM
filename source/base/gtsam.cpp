#include <base/gtsam.hpp>
namespace mvSLAM
{
gtsam::Point2
mvSLAM_Point2_to_gtsam_Point2(const mvSLAM::Point2 &p)
{
    return gtsam::Point2(p[0], p[1]);
}

mvSLAM::Point2
gtsam_Point2_to_mvSLAM_Point2(const gtsam::Point2 &p)
{
    return mvSLAM::Point2{p.x(), p.y()};
}

gtsam::Point3
mvSLAM_Point3_to_gtsam_Point3(const mvSLAM::Point3 &p)
{
    return gtsam::Point3(p[0], p[1], p[2]);
}

mvSLAM::Point3
gtsam_Point3_to_mvSLAM_Point3(const gtsam::Point3 &p)
{
    return mvSLAM::Point3{p.x(), p.y(), p.z()};
}

gtsam::Rot3
SO3_to_Rot3(const SO3 &p)
{
    auto m = p.get_matrix();
    gtsam::Matrix3 r;
    r << m(0, 0), m(0, 1), m(0, 2),
         m(1, 0), m(1, 1), m(1, 2),
         m(2, 0), m(2, 1), m(2, 2);
    return gtsam::Rot3(r);
}

SO3
Rot3_to_SO3(const gtsam::Rot3 &p)
{
    auto m = p.matrix();
    mvSLAM::Matrix3Type r;
    r << m(0, 0), m(0, 1), m(0, 2),
         m(1, 0), m(1, 1), m(1, 2),
         m(2, 0), m(2, 1), m(2, 2);
    return mvSLAM::SO3(r);
}

gtsam::Pose3
SE3_to_Pose3(const SE3 &p)
{
    auto r = SO3_to_Rot3(p.rotation());
    auto t = mvSLAM_Point3_to_gtsam_Point3(p.translation());
    return gtsam::Pose3(r, t);
}

SE3
Pose3_to_SE3(const gtsam::Pose3 &p)
{
    auto r = Rot3_to_SO3(p.rotation());
    auto t = gtsam_Point3_to_mvSLAM_Point3(p.translation());
    return mvSLAM::SE3(r, t);
}

}

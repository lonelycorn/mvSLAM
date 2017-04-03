#include <front-end/image-pair.hpp>
#include <vision/sfm.hpp>
#include <base/debug.hpp>
#include <base/image.hpp>

#include <algorithm>
#include <cassert>

namespace mvSLAM
{
static Logger logger("[ImagePair]", true);

ImagePair::ImagePair(const FrontEndTypes::FramePtr &base_frame_,
                     const FrontEndTypes::FramePtr &pair_frame_,
                     bool refine_structure):
    base_frame(base_frame_),
    pair_frame(pair_frame_),
    valid(false),

    match_inlier_count(0),
    match_inlier_ssd(-1), // delibrate overflow
    T_base_to_pair(),
    points(),

    error(infinity),
    T_base_to_pair_estimate(),
    point_estimates(),

    m_state(State::INIT),
    matches_base_to_pair(),
    point_indexes()
{
    assert(base_frame->id != pair_frame->id);
    logger.info("constructor, base frame id = ", base_frame_->id,
                ", pair frame id = ", pair_frame_->id);

    // a vector of {trainIdx, queryIdx, distance}
    matches_base_to_pair= \
        VisualFeature::match_visual_features(base_frame->visual_feature,
                                             pair_frame->visual_feature);

    logger.debug(matches_base_to_pair.size(), " matches found");

    reconstruct();

    if (valid && refine_structure)
    {
        refine();
    }
}

ImagePair::~ImagePair()
{
}

bool
ImagePair::update(const FrontEndTypes::FramePtr &new_frame)
{
    // perform light-weight reconstruction only
    ImagePair new_image_pair(base_frame, new_frame, false);

    if (!new_image_pair.valid)
    {
        return false;
    }

    // FIXME: is this true? does more inliers leads to better results?
    if ((new_image_pair.match_inlier_count < match_inlier_count) ||
        (new_image_pair.match_inlier_ssd < match_inlier_ssd))
    {
        return false;
    }

    // reconstruction didn't show any difference. need to refine
    new_image_pair.refine();
    if (new_image_pair.error < error)
    {
        std::swap(*this, new_image_pair);
        return true;
    }
    else
    {
        return false;
    }
}

bool
ImagePair::reconstruct()
{
    assert(State::INIT == m_state);

    size_t match_raw_count = matches_base_to_pair.size();

    // extract image points
    std::vector<ImagePoint> base_points;
    base_points.reserve(match_raw_count);
    {
        std::vector<ImagePoint> points = base_frame->visual_feature.get_image_points();
        for (const auto &m : matches_base_to_pair)
        {
            base_points.push_back(points[m.trainIdx]);
        }
    }
    std::vector<ImagePoint> pair_points;
    pair_points.reserve(match_raw_count);
    {
        std::vector<ImagePoint> points = pair_frame->visual_feature.get_image_points();
        for (const auto &m : matches_base_to_pair)
        {
            pair_points.push_back(points[m.queryIdx]);
        }
    }

    // reconstruct structure
    const CameraIntrinsics K = Matrix3Type::Identity(); // FIXME: get this info somewhere
    valid = sfm_solve(base_points,
                      pair_points,
                      K,
                      T_base_to_pair,
                      points,
                      point_indexes);
    if (!valid)
    {
        logger.info("reconstruct, failed");
    }
    else
    {
        match_inlier_count = point_indexes.size();
        for (auto idx : point_indexes)
        {
            const auto &m = matches_base_to_pair[idx];
            match_inlier_ssd += sqr(m.distance);
        }
        logger.debug("reconstruct, inliers count = ", match_inlier_count,
                     ", inlier ssd = ", match_inlier_ssd, ", T_base_to_pair =\n",
                     T_base_to_pair);
        m_state = State::RECONSTRUCTED;
    }
    return valid;
}

bool
ImagePair::refine()
{
    assert(State::RECONSTRUCTED == m_state);
    assert(valid);

    // extract point estimates
    std::vector<Point2Estimate> base_point_estimates;
    base_point_estimates.reserve(match_inlier_count);
    {
        std::vector<Point2Estimate> point_estimates = base_frame->visual_feature.get_point_estimates();
        for (auto idx : point_indexes)
        {
            const auto &m = matches_base_to_pair[idx];
            base_point_estimates.push_back(point_estimates[m.trainIdx]);
        }
    }
    std::vector<Point2Estimate> pair_point_estimates;
    pair_point_estimates.reserve(match_inlier_count);
    {
        std::vector<Point2Estimate> point_estimates = pair_frame->visual_feature.get_point_estimates();
        for (auto idx : point_indexes)
        {
            const auto &m = matches_base_to_pair[idx];
            pair_point_estimates.push_back(point_estimates[m.queryIdx]);
        }
    }

    const CameraIntrinsics K = Matrix3Type::Identity(); // FIXME: get this info somewhere
    valid = sfm_refine(base_point_estimates,
                       pair_point_estimates,
                       K,
                       T_base_to_pair,
                       points,
                       T_base_to_pair_estimate,
                       point_estimates,
                       error);
    if (!valid)
    {
        logger.info("refine, failed");
    }
    else
    {
        logger.debug("refine, error = ", error, ", T_base_to_pair_estimate.mean() =\n",
                     T_base_to_pair_estimate.mean());
        m_state = State::REFINED;
    }
    return valid;
}
}

#include <front-end/visual-odometer.hpp>
#include <front-end/image-pair.hpp>
#include <front-end/camera-manager.hpp>
#include <front-end/frame-manager.hpp>
#include <base/debug.hpp>
#include <vision/ba.hpp>
#include <vision/pnp.hpp>

#include <atomic>
#include <algorithm>
#include <cassert>
#include <unordered_set>

#define DEBUG_VISUAL_ODOMETER

namespace mvSLAM
{
static Logger logger("[VisualOdometer]");

static constexpr ScalarType VO_ANCHOR_STDDEV_POSITION(1e-3);
static constexpr ScalarType VO_ANCHOR_STDDEV_ORIENTATION(1e-3);
static constexpr ScalarType VO_REGULATOR_STDDEV_POSITION(1e-2);
static constexpr ScalarType VO_REGULATOR_STDDEV_ORIENTATION(1e-2);
static constexpr ScalarType VO_REGULATOR_STDDEV_POSITION_NEW(1e-1);
static constexpr ScalarType VO_REGULATOR_STDDEV_ORIENTATION_NEW(1e-1);

VisualOdometer::Params
VisualOdometer::get_default_params()
{
    // FIXME: magic numbers... magic numbers EVERYWHERE...
    Params p;

    p.max_match_inlier_distance = 10;

    p.frame_queue_size = 10;
    p.min_match_inlier_count = 20;
    p.max_error = 0.5;
    p.max_rotation_magnitude = 1e-1;
    p.max_translation_z = 0.1;

    p.min_tracked_point_match_count = 7;
    p.min_tracked_point_count = 10;

    return p;
}

VisualOdometer::VisualOdometer(const Params &p):
    m_mutex(),
    m_state(State::INITIALIZING),
    m_params(p),
    m_frame_queue(),
    m_image_pair_queue(),
    m_last_frame(nullptr),
    m_transformation(), // defaults to Identity
    m_point_id_to_point3(),
    m_point_id_to_last_frame_vf_idx(),
    m_last_frame_vf_idx_to_point_id()
{
    // check parameters validity
    assert(p.frame_queue_size >= 2);

    logger.info("constructor");
    reset();
}

VisualOdometer::~VisualOdometer()
{
    logger.info("destructor");
}

bool
VisualOdometer::add_frame(const FrontEndTypes::FramePtr &new_frame,
                          Transformation &T_new_frame_to_init)
{
    assert(new_frame && (new_frame->id != Id::INVALID));

    Lock lock(m_mutex);
    logger.info("add_frame, frame id = ", new_frame->id);

    bool found_transform = false;

    switch (m_state)
    {
    case State::INITIALIZING:
        if (initialize(new_frame))
        {
            found_transform = true;
        }
        break;
    case State::TRACKING:
        if (!track(new_frame))
        {
            // FIXME: not sure if this is appropriate
            reset();
            found_transform = false;
        }
        else
        {
            found_transform = true;
        }
        break;
    default:
        assert(false); // unknown state
    }

    if (found_transform)
    {
        T_new_frame_to_init = m_transformation;
    }
    return found_transform;
}

bool
VisualOdometer::add_frame_by_id(FrontEndTypes::FrameId new_frame_id,
                                Transformation &T_new_frame_to_init)
{
    auto new_frame = FrameManager::get_instance().get_frame(new_frame_id);
    return add_frame(new_frame, T_new_frame_to_init);
}

Transformation
VisualOdometer::get_transformation() const
{
    Lock lock(m_mutex);
    assert(initialized());
    return m_transformation;
}

void
VisualOdometer::reset()
{
    Lock lock(m_mutex);
    m_state = State::INITIALIZING;
    m_frame_queue.clear();
    m_image_pair_queue.clear();
    m_last_frame.reset();
    m_transformation = Transformation();
    m_point_id_to_point3.clear();
    m_point_id_to_last_frame_vf_idx.clear();
    m_last_frame_vf_idx_to_point_id.clear();
}

bool
VisualOdometer::initialized() const
{
    return (State::TRACKING == m_state);
}

bool
VisualOdometer::initialize(const FrontEndTypes::FramePtr &new_frame)
{
    assert(State::INITIALIZING == m_state);
    logger.info("initialize() new frame id = ", new_frame->id);

    // update all image pairs
    for (auto &ip : m_image_pair_queue)
    {
        bool updated = ip.update(new_frame);
        if (updated)
        {
            logger.debug("updated image pair, base frame id = ",
                         ip.base_frame->id);
        }
    }

    // try to create a new image pair with last frame
    if (m_frame_queue.size() > 0)
    {
        // reconstruct __AND__ refine the image pair
        auto image_pair_params = ImagePair::get_default_params();
        image_pair_params.max_match_inlier_distance = m_params.max_match_inlier_distance;
        image_pair_params.refine_structure_in_constructor = true;
        m_image_pair_queue.emplace_back(m_frame_queue.back(), new_frame, image_pair_params);
    }

    // append to frame queue
    m_frame_queue.push_back(new_frame);
    assert(m_frame_queue.size() == m_image_pair_queue.size() + 1);

#ifdef DEBUG_VISUAL_ODOMETER
    {
        logger.debug("m_frame_queue (size = ", m_frame_queue.size(), "):");
        for (auto &f : m_frame_queue)
        {
            logger.debug("\tframe id = ", f->id);
        }
        logger.debug("m_image_pair_queue (size = ", m_image_pair_queue.size(), "):");
        for (auto &ip : m_image_pair_queue)
        {
            logger.debug("\tvalid = ", ip.valid, ", base frame id = ", ip.base_frame->id,
                         ", pair frame id = ", ip.pair_frame->id);
        }
    }
#endif

    // check if any image pair is good for initialization
    for (auto &ip : m_image_pair_queue)
    {
        // iteration starts with the oldest image pair. In this way the first image pair
        // that passed the test should give us the longest baseline, which is good for
        // triangulation accuracy.
        // However, if other metrics are used, this may NOT be the best strategy.
        if (!check_image_pair(ip))
        {
            continue;
        }

        // now we have found a good image pair. initialize data structures for tracking
        m_last_frame = ip.pair_frame;
        assert(m_last_frame->id == new_frame->id);
        m_transformation = ip.T_base_to_pair_estimate.mean(); // from initialization to last frame
        // add all triangulated points
        size_t point_count = ip.point_estimates.size();
        for (size_t i = 0; i < point_count; ++i)
        {
            const auto &pe = ip.point_estimates[i];
            const auto vf_idx = ip.visual_feature_index_in_pair[i];

            // register point
            PointId id = generate_point_id();
            m_point_id_to_point3[id] = pe.mean(); // NOTE: throwing away the covar
            m_point_id_to_last_frame_vf_idx[id] = vf_idx;
            m_last_frame_vf_idx_to_point_id[vf_idx] = id;
        }

        // clear states used during initialization
        m_frame_queue.clear();
        m_image_pair_queue.clear();

        // advance to State::TRACKING
        m_state = State::TRACKING;

        // TODO: bundle adjustment on all frames in queue
        return true;
    }

    // check if need to dequeue
    if (m_frame_queue.size() > m_params.frame_queue_size)
    {
        assert(m_image_pair_queue.front().base_frame->id == m_frame_queue.front()->id);
        m_image_pair_queue.pop_front();
        m_frame_queue.pop_front();
    }

    return false;
}

bool
VisualOdometer::check_image_pair(const ImagePair &ip) const
{
    logger.debug("check image pair (base id = ", ip.base_frame->id, ", pair id = ", ip.pair_frame->id, ")");

    if (!ip.valid)
    {
        logger.debug("\timage pair not valid");
        return false;
    }
    if (ip.match_inlier_count < m_params.min_match_inlier_count)
    {
        logger.debug("\tnot enough match inliers: ", ip.match_inlier_count, " < ", m_params.min_match_inlier_count);
        return false;
    }
    if (ip.error > m_params.max_error)
    {
        logger.debug("\tlarge refinement error: ", ip.error, " > ", m_params.max_error);
        return false;
    }
    Vector3Type so3 = ip.T_base_to_pair_estimate.mean().rotation().ln();
    if (so3.squaredNorm() > sqr(m_params.max_rotation_magnitude))
    {
        logger.debug("\tlarge rotation magnitude: ", so3.norm(), " > ", m_params.max_rotation_magnitude);
        return false;
    }
    Vector3Type t = ip.T_base_to_pair_estimate.mean().translation();
    if (std::abs(t[2]) > m_params.max_translation_z)
    {
        logger.debug("\tlarge z-translation: ", std::abs(t[2]), " > ", m_params.max_translation_z);
        return false;
    }
    logger.debug("\tpassed all checks");
    return true;
}

bool
VisualOdometer::track(const FrontEndTypes::FramePtr &new_frame)
{
    assert(State::TRACKING == m_state);

    // NOTE: light-weight; no refinement
    auto image_pair_params = ImagePair::get_default_params();
    image_pair_params.max_match_inlier_distance = m_params.max_match_inlier_distance;
    ImagePair ip(m_last_frame, new_frame, image_pair_params);

    // previously tracked points that could be found in new frame
    std::unordered_map<PointId, size_t> tracked_point_id_to_new_frame_vf_idx;
    //std::unordered_map<size_t, PointId> new_frame_vf_idx_to_tracked_point_id;
    {
        for (size_t i = 0; i < ip.points.size(); ++i)
        {
            auto vf_idx_last_frame = ip.visual_feature_index_in_base[i];
            if (m_last_frame_vf_idx_to_point_id.count(vf_idx_last_frame) == 0)
            {
                continue;
            }
            auto vf_idx_new_frame = ip.visual_feature_index_in_pair[i];
            auto pid = m_last_frame_vf_idx_to_point_id.at(vf_idx_last_frame);
            tracked_point_id_to_new_frame_vf_idx[pid] = vf_idx_new_frame;
            //new_frame_vf_idx_to_tracked_point_id[vf_idx_new_frame] = pid;
        }

        if (tracked_point_id_to_new_frame_vf_idx.size() < m_params.min_tracked_point_count)
        {
            logger.debug("not enough tracked points found in new frame: ",
                         tracked_point_id_to_new_frame_vf_idx.size());
            return false;
        }
    }

    // PnP to find new frame pose
    Transformation new_frame_transformation; // T from new ref frame to init ref frame
    std::unordered_set<PointId> outlier_point_id;
    {
        // ID of all tracked points
        std::vector<PointId> point_id;
        point_id.reserve(tracked_point_id_to_new_frame_vf_idx.size());

        // tracked points in world ref frame.
        std::vector<Point3> matched_world_points;
        matched_world_points.reserve(tracked_point_id_to_new_frame_vf_idx.size());

        // image of tracked points
        std::vector<ImagePoint> new_frame_image_points = \
            new_frame->visual_feature.get_image_points();
        std::vector<ImagePoint> matched_image_points;
        matched_image_points.reserve(tracked_point_id_to_new_frame_vf_idx.size());

        for (const auto &p : tracked_point_id_to_new_frame_vf_idx)
        {
            auto pid = p.first;
            auto vf_idx = p.second;
            point_id.push_back(pid);
            matched_world_points.push_back(m_point_id_to_point3[pid]);
            matched_image_points.push_back(new_frame_image_points[vf_idx]);
            outlier_point_id.insert(pid);
        }

        std::vector<size_t> inlier_point_indexes;
        if (!pnp_solve(matched_world_points,
                       matched_image_points,
                       CameraManager::get_camera().get_intrinsics(),
                       new_frame_transformation,
                       inlier_point_indexes))
        {
            logger.info("cannot track new frame pose.");
            return false;
        }
        logger.debug("PnP found ", inlier_point_indexes.size(), " inliers; new frame transformation:\n", new_frame_transformation);

        // remove inliers from outlier_point_id
        for (auto idx : inlier_point_indexes)
        {
            outlier_point_id.erase(point_id[idx]);
        }
    }

    // find out scale of the new image pair reconstruction
    // point_in_init_ref_frame = T_base_ref_frame_to_init_ref_frame * (point_in_base_ref_frame * scale)
    ScalarType scale(0.0);
    {
        // FIXME: we should also use the point location to calculate the scale, in a similar way to poin registration.
        auto T_new_frame_to_last_frame = new_frame_transformation * m_transformation.inverse();
        scale = T_new_frame_to_last_frame.translation().norm();
        logger.debug("scale = ", scale);
    }

    // data structure related to newly triangulated points
    std::unordered_map<PointId, Point3> new_point_id_to_point3;
    std::unordered_map<PointId, size_t> new_point_id_to_new_frame_vf_idx;
    std::unordered_map<PointId, size_t> new_point_id_to_last_frame_vf_idx;
    std::unordered_map<size_t, PointId> new_frame_vf_idx_to_new_point_id;
    {

        for (size_t i = 0; i < ip.points.size(); ++i)
        {
            const auto &point_in_base_ref_frame = ip.points[i];
            auto vf_idx_in_base = ip.visual_feature_index_in_base[i];
            auto vf_idx_in_pair = ip.visual_feature_index_in_pair[i];
            PointId pid = Id::INVALID;
            if (m_last_frame_vf_idx_to_point_id.count(vf_idx_in_base) == 0) // this is a newly triangulated point
            {
                pid = generate_point_id();
                new_point_id_to_point3[pid] = m_transformation * (point_in_base_ref_frame * scale);
            }
            else
            {
                pid = m_last_frame_vf_idx_to_point_id[vf_idx_in_base];
                if (outlier_point_id.count(pid) > 0) // skip outlier points
                {
                    continue;
                }
                new_point_id_to_point3[pid] = m_point_id_to_point3[pid];
            }
            new_point_id_to_new_frame_vf_idx[pid] = vf_idx_in_pair;
            new_point_id_to_last_frame_vf_idx[pid] = vf_idx_in_base;
            new_frame_vf_idx_to_new_point_id[vf_idx_in_pair] = pid;
        }
    }

#ifdef DEBUG_VISUAL_ODOMETER
    /*
    logger.debug(m_point_id_to_point3.size(), " existing tracked points");
    for (const auto &p : m_point_id_to_point3)
    {
        auto pid = p.first;
        const auto &point3 = p.second;
        logger.debug("===== point id = ", pid, "=====\n", point3);
    }
    logger.debug(m_point_id_to_point3.size(), " new tracked points");
    for (const auto &p : new_point_id_to_point3)
    {
        auto pid = p.first;
        const auto &point3 = p.second;
        if (m_point_id_to_point3.count(pid) > 0) // a previously tracked point
        {
            continue;
        }
        logger.debug("===== point id = ", pid, "=====\n", point3);
    }
    */
#endif

    // bundle-adjustment
    const CameraIntrinsics &ci = CameraManager::get_camera().get_intrinsics();
    std::unordered_set<Id::Type> frame_id;
    std::unordered_set<Id::Type> point_id;
    std::unordered_map<Id::Type, Transformation> frame_pose_guess;
    std::unordered_map<Id::Type, TransformationUncertainty> frame_pose_prior;
    const std::unordered_map<Id::Type, Point3> &point_guess = new_point_id_to_point3; // should have everything
    std::unordered_map<Id::Type, Point3Uncertainty> point_prior;
    std::unordered_map<Id::Type, PointIdToPoint2Estimate> frame_observation;
    std::unordered_map<Id::Type, TransformationEstimate> frame_pose_estimate;
    std::unordered_map<Id::Type, Point3Estimate> point_estimate;
    ScalarType final_error;

    // frame id
    frame_id.insert(m_last_frame->id);
    frame_id.insert(new_frame->id);

    // point id
    for (const auto &p : new_point_id_to_point3)
    {
        auto pid = p.first;
        point_id.insert(pid);
    }

    // frame pose guess
    frame_pose_guess[m_last_frame->id] = m_transformation;
    frame_pose_guess[new_frame->id] = new_frame_transformation;

    // frame pose prior
    {
        // anchor last frame pose
        // FIXME: no regulator for new_frame
        Matrix6Type covar = TransformationUncertainty::Identity();
        for (int i = 0; i < 3; ++i)
        {
            covar(i, i) *= VO_ANCHOR_STDDEV_POSITION;
            covar(i+3, i+3) *= VO_ANCHOR_STDDEV_ORIENTATION;
        }
        frame_pose_prior[m_last_frame->id] = covar;
    }

    // point guess is the same as new_point_id_to_point3

    // point prior
    for (const auto &p : tracked_point_id_to_new_frame_vf_idx)
    {
        // regulate previously tracked points
        // FIXME: no regulator for newly triangulated points
        auto pid = p.first;
        auto covar = Point3Uncertainty::Identity() * sqr(VO_REGULATOR_STDDEV_POSITION);
        point_prior[pid] = covar;
    }

    // frame observation
    for (int camera_index = 0; camera_index < 2; ++camera_index)
    {
        const auto fid = (camera_index == 0 ?
                m_last_frame->id :
                new_frame->id);
        const auto point2estimate = (camera_index == 0 ?
                m_last_frame->visual_feature.get_point_estimates() :
                new_frame->visual_feature.get_point_estimates());
        const auto &point_id_to_vf_idx = (camera_index == 0 ?
                new_point_id_to_last_frame_vf_idx :
                new_point_id_to_new_frame_vf_idx);

        PointIdToPoint2Estimate observation;
        for (const auto &p : point_id_to_vf_idx)
        {
            auto pid = p.first;
            auto vf_idx = p.second;
            const auto &pe = point2estimate[vf_idx];
            observation[pid] = pe;
        }
        frame_observation[fid] = observation;
    }


    ba_frame_pose_and_point(ci,
                            frame_id,
                            point_id,
                            frame_pose_guess,
                            frame_pose_prior,
                            point_guess,
                            point_prior,
                            frame_observation,
                            frame_pose_estimate,
                            point_estimate,
                            final_error);

    logger.debug("ba final error = ", final_error, ", refined transformation =\n",
            frame_pose_estimate[new_frame->id].mean());
    if (final_error > m_params.max_error)
    {
        logger.debug("large refinement error: ", final_error, " > ", m_params.max_error);
        return false;
    }

    for (auto &p : new_point_id_to_point3)
    {
        auto pid = p.first;
        // TODO: maybe reject points that very uncertain?
        p.second = point_estimate[pid].mean();
    }
    // NOTE: throwing away the covar
    new_frame_transformation = frame_pose_estimate[new_frame->id].mean();

    // update internal states
    m_last_frame = new_frame;
    std::swap(m_transformation, new_frame_transformation);
    std::swap(m_point_id_to_point3, new_point_id_to_point3);
    std::swap(m_point_id_to_last_frame_vf_idx, new_point_id_to_new_frame_vf_idx);
    m_last_frame_vf_idx_to_point_id.swap(new_frame_vf_idx_to_new_point_id);
    return true;
}

VisualOdometer::PointId
VisualOdometer::generate_point_id()
{
    // NOTE: this is shared among ALL instances of VisualOdometer.
    static std::atomic<PointId> next_id(0);
    return next_id.fetch_add(1);
}
}

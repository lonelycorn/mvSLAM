#include <front-end/visual-odometer.hpp>
#include <front-end/image-pair.hpp>
#include <front-end/camera-manager.hpp>
#include <front-end/frame-manager.hpp>
#include <base/parameter-manager.hpp>
#include <base/debug.hpp>
#include <vision/ba.hpp>
#include <vision/pnp.hpp>

#include <atomic>
#include <algorithm>
#include <cassert>
#include <unordered_set>
#include <string>

//#define DEBUG_VISUAL_ODOMETER

namespace mvSLAM
{
static const std::string module_name("VisualOdometer");
static Logger logger("[VisualOdometer]");

template <typename T>
static std::string
debug_stringize(const T &t)
{
    std::stringstream ss;
    for (const auto &i : t)
    {
        ss << " " << i;
    }
    return ss.str();
}

template <typename T>
static std::string
debug_stringize(const std::unordered_set<T> &s)
{
    std::vector<T> tmp;
    tmp.reserve(s.size());
    for (const auto &i : s)
    {
        tmp.push_back(i);
    }
    std::sort(tmp.begin(), tmp.end());
    return debug_stringize(tmp);
}

template <typename T1, typename T2>
static std::string
debug_stringize(const std::unordered_map<T1, T2> &m)
{
    std::vector<T1> tmp;
    tmp.reserve(m.size());
    for (const auto &i : m)
    {
        tmp.push_back(i.first);
    }
    std::sort(tmp.begin(), tmp.end());
    return debug_stringize(tmp);
}

VisualOdometer::Params
VisualOdometer::get_default_params()
{
    Params p;

    p.max_match_inlier_distance = ParameterManager::get_value<int>(
            module_name, "max_match_inlier_distance", 10);

    p.frame_queue_size = ParameterManager::get_value<int>(
            module_name, "frame_queue_size", 10);

    p.min_match_inlier_count = ParameterManager::get_value<int>(
            module_name, "min_match_inlier_count", 20);

    p.max_error = ParameterManager::get_value<ScalarType>(
            module_name, "max_error", 0.5);

    p.max_rotation_magnitude = ParameterManager::get_value<ScalarType>(
            module_name, "max_rotation_magnitude", 0.1);

    p.max_translation_z = ParameterManager::get_value<ScalarType>(
            module_name, "max_translation_z", 0.1);

    p.min_pnp_point_count = ParameterManager::get_value<int>(
            module_name, "min_pnp_point_count", 7);

    p.min_tracked_point_count = ParameterManager::get_value<int>(
            module_name, "min_tracked_point_count", 10);

    p.anchor_stddev_position = ParameterManager::get_value<ScalarType>(
            module_name, "anchor_stddev_position", 1e-3);

    p.anchor_stddev_orientation = ParameterManager::get_value<ScalarType>(
            module_name, "anchor_stddev_orientation", 1e-3);

    p.regulator_stddev_position = ParameterManager::get_value<ScalarType>(
            module_name, "regulator_stddev_position", 1e-2);

    p.regulator_stddev_orientation = ParameterManager::get_value<ScalarType>(
            module_name, "regulator_stddev_orientation", 1e-2);

    return p;
}

VisualOdometer::VisualOdometer(const Params &p):
    m_params(p),
    m_mutex(),
    m_state(State::INITIALIZING),
    m_frame_queue(),
    m_image_pair_queue(),
    T_last_frame_to_init_frame(), // defaults to Identity
    m_tracked_points(),
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
VisualOdometer::add_frame(const FrontEndTypes::FramePtr &new_frame)
{
    assert(new_frame && (new_frame->id != Id::INVALID));

    Lock lock(m_mutex);
    logger.info("add_frame, frame id = ", new_frame->id);

    // update the frame queue
    m_frame_queue.push_back(new_frame);
    // create a new image pair from the last two frames
    if (m_frame_queue.size() > 1)
    {
        const auto &prev_frame = *(m_frame_queue.crbegin() + 1);
        // reconstruction only
        auto image_pair_params = ImagePair::get_default_params();
        image_pair_params.max_match_inlier_distance = m_params.max_match_inlier_distance;
        image_pair_params.refine_structure_in_constructor = false;
        m_image_pair_queue.emplace_back(prev_frame, new_frame, image_pair_params);
    }
    assert(m_frame_queue.size() == m_image_pair_queue.size() + 1);

    bool found_transform = false;

    switch (m_state)
    {
    case State::INITIALIZING:
        if (initialize())
        {
            found_transform = true;

            // advance state
            m_state = State::TRACKING;
        }
        break;
    case State::TRACKING:
        if (!track())
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

    // check if need to remove outdated frames and image pairs
    if (m_frame_queue.size() > m_params.frame_queue_size)
    {
        assert(m_image_pair_queue.size() > 0);
        assert(m_image_pair_queue.front().base_frame->id == m_frame_queue.front()->id);
        logger.debug("popped outdated frame (id = ", m_frame_queue.front()->id, ")");
        m_frame_queue.pop_front();
        logger.debug("popped outdated ImagePair,base frame id = ",
                m_image_pair_queue.front().base_frame->id, ")");
        m_image_pair_queue.pop_front();
    }
    assert(m_frame_queue.size() == m_image_pair_queue.size() + 1);

    return found_transform;
}

bool
VisualOdometer::add_frame_by_id(FrontEndTypes::FrameId new_frame_id)
{
    auto new_frame = FrameManager::get_frame(new_frame_id);
    return add_frame(new_frame);
}

void
VisualOdometer::reset()
{
    Lock lock(m_mutex);
    m_state = State::INITIALIZING;

    while (m_frame_queue.size() > 1) // keep the last frame
    {
        m_frame_queue.pop_front();
    }
    m_image_pair_queue.clear();
    T_last_frame_to_init_frame = Transformation();
    m_tracked_points.clear();
    m_last_frame_vf_idx_to_point_id.clear();
}

bool
VisualOdometer::initialized() const
{
    return (State::TRACKING == m_state);
}

Transformation
VisualOdometer::get_body_pose() const
{
    Lock lock(m_mutex);
    assert(initialized());
    // T_B2_to_W = T_B1_to_W * T_B2_to_B1  body
    // T_C2_to_W = T_C1_to_W * T_C2_to_C1  camera
    // T_C1_to_W = T_B1_to_W * T_C_to_B
    // T_C2_to_W = T_B2_to_W * T_C_to_B
    //
    // let B1, C1 be the corresponding ref frames of last initialization,
    // and B2, C2 the corresponding ref frames of latest frame, we have
    // T_C2_to_C1 = T_last_frame_to_init_frame
    // T_B1_to_W = Identity
    // T_B2_to_W = T_B1_to_W * T_C_to_B * T_C2_to_C1 * T_B_to_C
    //           = T_B_to_C.inverse() * T_C2_to_C1 * T_B_to_C
    const auto &P = CameraManager::get_camera().get_extrinsics();
    const auto &P_inv = CameraManager::get_camera().get_extrinsics_inverse();
    return P_inv * T_last_frame_to_init_frame * P;
}

Transformation
VisualOdometer::get_camera_pose() const
{
    Lock lock(m_mutex);
    assert(initialized());
    const auto &P_inv = CameraManager::get_camera().get_extrinsics_inverse();
    return P_inv * T_last_frame_to_init_frame;
}

std::vector<Point3>
VisualOdometer::get_tracked_points() const
{
    Lock lcok(m_mutex);
    assert(initialized());

    const auto &P_inv = CameraManager::get_camera().get_extrinsics_inverse();
    std::vector<Point3> result;
    result.reserve(m_tracked_points.size());
    for (const auto &p : m_tracked_points)
    {
        const auto &position = p.second.position;
        result.emplace_back(P_inv * position);
    }

    return result;
}

bool
VisualOdometer::initialize()
{
    assert(State::INITIALIZING == m_state);
    assert(m_frame_queue.size() > 0);

    const FrontEndTypes::FramePtr &new_frame = m_frame_queue.back();
    logger.info("initialize() new frame id = ", new_frame->id);

    // refine the last image pair
    if (m_image_pair_queue.size() > 0)
    {
        m_image_pair_queue.back().refine();
    }

    // update all image pairs
    for (auto &ip : m_image_pair_queue)
    {
        bool updated = ip.update(new_frame);
        if (updated)
        {
            logger.debug("updated image pair, base frame id = ", ip.base_frame->id);
        }
    }

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
    for (const auto &ip : m_image_pair_queue)
    {
        // NOTE: the iteration starts with the oldest image pair. In this way the first
        //       image pair that passes the test should have the longest baseline, which
        //       is good for triangulation accuracy. However, if other metrics are used,
        //       this may __NOT__ be the best strategy.
        if (!check_image_pair(ip))
        {
            continue;
        }

        // now we have found a good image pair. initialize data structures for tracking
        assert(ip.pair_frame->id == new_frame->id);
        T_last_frame_to_init_frame = ip.T_pair_to_base;
        m_tracked_points.reserve(ip.matched_points.size());
        m_last_frame_vf_idx_to_point_id.reserve(ip.matched_points.size());
        for (const auto &mp : ip.matched_points)
        {
            // register tracked point
            // NOTE: throwing away the covar
            PointId id = generate_point_id();
            TrackedPoint tp(id, mp.position, mp.vf_idx_in_pair);
            m_tracked_points.emplace(id, tp);
            m_last_frame_vf_idx_to_point_id.emplace(mp.vf_idx_in_pair, id);
        }

        // TODO: bundle adjustment on all frames in queue
        return true;
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
    Vector3Type so3 = ip.T_pair_to_base.rotation().ln();
    if (so3.squaredNorm() > sqr(m_params.max_rotation_magnitude))
    {
        logger.debug("\tlarge rotation magnitude: ", so3.norm(), " > ", m_params.max_rotation_magnitude);
        return false;
    }
    Vector3Type t = ip.T_pair_to_base.translation();
    if (std::abs(t[2]) > m_params.max_translation_z)
    {
        logger.debug("\tlarge z-translation: ", std::abs(t[2]), " > ", m_params.max_translation_z);
        return false;
    }
    logger.debug("\tpassed all checks");
    return true;
}

bool
VisualOdometer::track()
{
    assert(State::TRACKING == m_state);

    // NOTE: only light-weight reconstruction were performed
    assert(m_image_pair_queue.size() > 0);
    const auto &ip = m_image_pair_queue.back();

    // PnP for new frame pose
    std::unordered_map<PointId, TrackedPoint> tracked_points;
    std::unordered_map<size_t, PointId> new_frame_vf_idx_to_point_id;
    std::unordered_map<size_t, PointId> last_frame_vf_idx_to_point_id;
    Transformation T_new_frame_to_init_frame;
    ScalarType scale;

    if (!track_pnp(ip,
                   tracked_points,
                   new_frame_vf_idx_to_point_id,
                   T_new_frame_to_init_frame,
                   scale))
    {
        return false;
    }

    if (tracked_points.size() < m_params.min_pnp_point_count)
    {
        logger.debug("not enough PnP points found in new frame: ",
                     tracked_points.size(), " < ",
                     m_params.min_pnp_point_count);
        return false;
    }

    // add newly triangulated points.
    // NOTE: pnp points have already been added.
    // FIXME: indeed we should check if the matched points fit the essential matrix
    new_frame_vf_idx_to_point_id.reserve(ip.matched_points.size());
    last_frame_vf_idx_to_point_id.reserve(ip.matched_points.size());
    for (const auto &mp : ip.matched_points)
    {
        const auto &unscaled_position = mp.position; // position in base frame
        auto vf_idx_new_frame = mp.vf_idx_in_pair;
        auto vf_idx_last_frame = mp.vf_idx_in_base;

        // this vf idx has not been registered before.
        // NOTE: this also rejects PnP inlier and outlier points
        if (m_last_frame_vf_idx_to_point_id.count(vf_idx_last_frame) == 0)
        {
            auto pid = generate_point_id();
            Point3 position_in_init_ref_frame = T_last_frame_to_init_frame *
                (unscaled_position * scale);
            TrackedPoint tp(pid, position_in_init_ref_frame, vf_idx_new_frame);
            tracked_points.emplace(pid, tp);
            new_frame_vf_idx_to_point_id.emplace(vf_idx_new_frame, pid);
            last_frame_vf_idx_to_point_id.emplace(vf_idx_last_frame, pid);

            // FIXME: do we really need following? would it be possible for the same
            //        last frame vf idx be matched against many new frame vf idx?
            auto tmp = m_last_frame_vf_idx_to_point_id.emplace(vf_idx_last_frame, pid);
            assert(tmp.second); // if this fired, we definitely made a false assumption
        }
    }

#ifdef DEBUG_VISUAL_ODOMETER
    /*
    logger.debug(m_tracked_points.size(), " existing tracked points");
    for (const auto &p : m_tracked_points)
    {
        auto pid = p.first;
        const auto &point3 = p.second.position;
        logger.debug("===== point id = ", pid, "=====\n", point3);
    }
    logger.debug(tracked_points.size(), " new tracked points");
    for (const auto &p : tracked_points)
    {
        auto pid = p.first;
        const auto &point3 = p.second.position;
        logger.debug("===== point id = ", pid, "=====\n", point3);
    }
    */
#endif

    // Bundle adjustment to refine frame pose and point positions
    std::unordered_map<PointId, Point3Estimate> tracked_point_estimate;
    TransformationEstimate T_new_frame_to_init_frame_estimate;
    ScalarType final_error;
    track_refine(ip,
                 tracked_points,
                 new_frame_vf_idx_to_point_id,
                 last_frame_vf_idx_to_point_id,
                 T_new_frame_to_init_frame,
                 tracked_point_estimate,
                 T_new_frame_to_init_frame_estimate,
                 final_error);

    if (final_error > m_params.max_error)
    {
        logger.debug("large refinement error: ", final_error, " > ", m_params.max_error);
        return false;
    }

    // update tracked points in new frame. NOTE: throwing away the covar
    for (auto &p : tracked_points)
    {
        auto pid = p.first;
        auto &tp = p.second;
        // TODO: maybe reject points that very uncertain?
        tp.position = tracked_point_estimate.at(pid).mean();
    }
    T_new_frame_to_init_frame = T_new_frame_to_init_frame_estimate.mean();

    // update internal states
    std::swap(T_last_frame_to_init_frame, T_new_frame_to_init_frame);
    std::swap(m_tracked_points, tracked_points);
    std::swap(m_last_frame_vf_idx_to_point_id, new_frame_vf_idx_to_point_id);
    return true;
}

bool
VisualOdometer::track_pnp(
        const ImagePair &ip,
        std::unordered_map<PointId, TrackedPoint> &pnp_tracked_points_,
        std::unordered_map<size_t, PointId> &pnp_new_frame_vf_idx_to_point_id_,
        Transformation &T_new_frame_to_init_frame_,
        ScalarType &scale_) const
{

    // ID of tracked points that are found in the new frame.
    std::vector<PointId> candidate_point_id;
    candidate_point_id.reserve(m_tracked_points.size());

    // vf idx in new frame for the corresponding candidate PnP point
    std::vector<size_t> candidate_point_vf_idx_new_frame;
    candidate_point_vf_idx_new_frame.reserve(m_tracked_points.size());

    for (const auto &mp : ip.matched_points)
    {
        auto vf_idx_last_frame = mp.vf_idx_in_base;
        if (m_last_frame_vf_idx_to_point_id.count(vf_idx_last_frame) > 0)
        {
            auto pid = m_last_frame_vf_idx_to_point_id.at(vf_idx_last_frame);
            auto vf_idx_new_frame = mp.vf_idx_in_pair;
            candidate_point_id.push_back(pid);
            candidate_point_vf_idx_new_frame.push_back(vf_idx_new_frame);
        }
    }
    const auto candidate_point_count = candidate_point_id.size();

    // PnP to find new frame pose. also find out inlier PnP points
    Transformation T_new_frame_to_init_frame; // "camera" to "world"
    std::unordered_set<PointId> inlier_point_id;
    {
        // tracked points in world ref frame.
        std::vector<Point3> matched_world_points;
        matched_world_points.reserve(candidate_point_count);

        // image of tracked points
        std::vector<ImagePoint> matched_image_points;
        matched_image_points.reserve(candidate_point_count);

        // subscipts of 3D-2D correspondences that are inliers
        std::vector<size_t> inlier_subscripts;

        std::vector<ImagePoint> new_frame_image_points = \
            ip.pair_frame->visual_feature.get_image_points();
        for (size_t i = 0; i < candidate_point_count; ++i)
        {
            auto pid = candidate_point_id.at(i);
            auto vf_idx_new_frame = candidate_point_vf_idx_new_frame.at(i);
            matched_world_points.push_back(m_tracked_points.at(pid).position);
            matched_image_points.push_back(new_frame_image_points.at(vf_idx_new_frame));
        }

        if (!pnp_solve(matched_world_points,
                       matched_image_points,
                       CameraManager::get_camera().get_intrinsics(),
                       T_new_frame_to_init_frame,
                       inlier_subscripts))
        {
            logger.info("cannot track new frame pose.");
            return false;
        }

        logger.debug("PnP found ", inlier_subscripts.size(), " inliers;",
                     " new frame transformation:\n", T_new_frame_to_init_frame);

        // mark inliers
        inlier_point_id.reserve(inlier_subscripts.size());
        for (auto subscript : inlier_subscripts)
        {
            auto pid = candidate_point_id.at(subscript);
            inlier_point_id.insert(pid);
        }
    }

    // find out scale of the image pair reconstruction in the init ref frame
    ScalarType scale(0.0);
    {
        // FIXME: we should also use the point location to calculate the scale,
        //        in a similar way to point registration.
        Transformation T_new_frame_to_last_frame = T_last_frame_to_init_frame.inverse() *
                T_new_frame_to_init_frame;
        // NOTE: image_pair.T_pair_to_base.translation().norm() == 1
        scale = T_new_frame_to_last_frame.translation().norm();
        logger.debug("scale = ", scale);
    }

    // prepare for output
    pnp_tracked_points_.reserve(inlier_point_id.size());
    pnp_new_frame_vf_idx_to_point_id_.reserve(inlier_point_id.size());
    for (size_t i = 0; i < candidate_point_count; ++i)
    {
        auto pid = candidate_point_id.at(i);
        auto vf_idx_new_frame = candidate_point_vf_idx_new_frame.at(i);
        if (inlier_point_id.count(pid) > 0)
        {
            const auto &position = m_tracked_points.at(pid).position;
            TrackedPoint tp(pid, position, vf_idx_new_frame);
            pnp_tracked_points_.emplace(pid, tp);
            pnp_new_frame_vf_idx_to_point_id_.emplace(vf_idx_new_frame, pid);
        }
    }
    std::swap(T_new_frame_to_init_frame_, T_new_frame_to_init_frame);
    std::swap(scale_, scale);
#ifdef DEBUG_VISUAL_ODOMETER
    {
        logger.debug("pnp inlier point:\n\t", debug_stringize(inlier_point_id));
    }
#endif

    return true;
}

void
VisualOdometer::track_refine(
        const ImagePair &ip,
        const std::unordered_map<PointId, TrackedPoint> &tracked_points,
        const std::unordered_map<size_t, PointId> &new_frame_vf_idx_to_point_id,
        const std::unordered_map<size_t, PointId> &last_frame_vf_idx_to_point_id,
        const Transformation &T_new_frame_to_init_frame,
        std::unordered_map<PointId, Point3Estimate> &tracked_point_estimate,
        TransformationEstimate &T_new_frame_to_init_frame_estimate,
        ScalarType &final_error) const
/*
void
VisualOdometer::track_refine(
        const ImagePair &ip,
        const std::unordered_map<PointId, TrackedPoint> &tracked_points,
        const std::unordered_map<size_t, PointId> &new_frame_vf_idx_to_point_id,
        const std::unordered_map<size_t, PointId> &last_frame_vf_idx_to_point_id,
        const Transformation &T_new_frame_to_init_frame,
        std::unordered_map<PointId, Point3> &tracked_point_estimate,
        TransformationEstimate &T_new_frame_to_init_frame_estimate,
        ScalarType &final_error) const
        */
{
    const auto new_frame = ip.pair_frame;
    const auto last_frame = ip.base_frame;

    // bundle-adjustment
    const CameraIntrinsics &ci = CameraManager::get_camera().get_intrinsics();
    std::unordered_set<Id::Type> frame_id;
    std::unordered_set<Id::Type> point_id;
    std::unordered_map<Id::Type, Transformation> frame_pose_guess;
    std::unordered_map<Id::Type, TransformationUncertainty> frame_pose_prior;
    std::unordered_map<Id::Type, Point3> point_guess;
    std::unordered_map<Id::Type, Point3Uncertainty> point_prior;
    std::unordered_map<Id::Type, PointIdToPoint2Estimate> frame_observation;
    std::unordered_map<Id::Type, TransformationEstimate> frame_pose_estimate;
    std::unordered_map<Id::Type, Point3Estimate> point_estimate;

    // frame id
    {
        frame_id.reserve(2);
        frame_id.insert(last_frame->id);
        frame_id.insert(new_frame->id);
    }

    // point id
    {
        point_id.reserve(tracked_points.size());
        for (const auto &p : tracked_points)
        {
            auto pid = p.first;
            point_id.insert(pid);
        }
    }

    // frame pose guess
    {
        frame_pose_guess.reserve(2);
        frame_pose_guess.emplace(last_frame->id, T_last_frame_to_init_frame);
        frame_pose_guess.emplace(new_frame->id, T_new_frame_to_init_frame);
    }

    // frame pose prior
    {
        frame_pose_prior.reserve(2);

        // anchor last frame pose
        Matrix6Type covar = TransformationUncertainty::Identity();
        for (int i = 0; i < 3; ++i)
        {
            covar(i, i) *= m_params.anchor_stddev_position;
            covar(i+3, i+3) *= m_params.anchor_stddev_orientation;
        }
        frame_pose_prior.emplace(last_frame->id, covar);

        // regulate new frame pose
        covar = TransformationUncertainty::Identity();
        for (int i = 0; i < 3; ++i)
        {
            covar(i, i) *= m_params.regulator_stddev_position;
            covar(i+3, i+3) *= m_params.regulator_stddev_orientation;
        }
        frame_pose_prior.emplace(new_frame->id, covar);
    }

    // point guess
    {
        point_guess.reserve(tracked_points.size());
        for (const auto &p : tracked_points)
        {
            auto pid = p.first;
            const auto &position = p.second.position;
            point_guess.emplace(pid, position);
        }
    }

    // point prior
    {
        point_prior.reserve(tracked_points.size());
        for (const auto &p : tracked_points)
        {
            auto pid = p.first;
            bool is_new_point = (m_tracked_points.count(pid) == 0);
            if (is_new_point)
            {
                // NOTE: no regulations for new points because we don't have
                //       confidence in them
                continue;
            }
            else
            {
                Point3Uncertainty covar = Point3Uncertainty::Identity() *
                    sqr(m_params.regulator_stddev_position);
                point_prior.emplace(pid, covar);
            }

        }
    }

    // frame observation
    frame_observation.reserve(2);
    {
        auto fid = last_frame->id;
        const auto point2estimate = last_frame->visual_feature.get_point_estimates();

        PointIdToPoint2Estimate observation;
        observation.reserve(last_frame_vf_idx_to_point_id.size());

        for (const auto &p : last_frame_vf_idx_to_point_id)
        {
            auto vf_idx = p.first;
            auto pid = p.second;
            const auto &pe = point2estimate.at(vf_idx);
            assert(point_id.count(pid) > 0);
            observation.emplace(pid, pe);
        }
        frame_observation.emplace(fid, observation);
    }
    {
        PointIdToPoint2Estimate observation;
        observation.reserve(new_frame_vf_idx_to_point_id.size());

        auto fid = new_frame->id;
        const auto point2estimate = new_frame->visual_feature.get_point_estimates();

        for (const auto &p : new_frame_vf_idx_to_point_id)
        {
            auto vf_idx = p.first;
            auto pid = p.second;
            const auto &pe = point2estimate.at(vf_idx);
            assert(point_id.count(pid) > 0);
            observation.emplace(pid, pe);
        }
        frame_observation.emplace(fid, observation);
    }

#ifdef DEBUG_VISUAL_ODOMETER
    {
        logger.debug("frame_id:\n\t", debug_stringize(frame_id));
        logger.debug("point_id:\n\t", debug_stringize(point_id));
        logger.debug("frame_pose_guess:\n\t", debug_stringize(frame_pose_guess));
        logger.debug("frame_pose_prior:\n\t", debug_stringize(frame_pose_prior));
        logger.debug("point_guess:\n\t", debug_stringize(point_guess));
        logger.debug("point_prior:\n\t", debug_stringize(point_prior));
        for (const auto &p : frame_observation)
        {
            logger.debug("frame ", p.first, " observation:\n\t", debug_stringize(p.second));
        }
    }
#endif
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
            frame_pose_estimate.at(new_frame->id).mean());

    // prepare for output
    std::swap(tracked_point_estimate, point_estimate);
    std::swap(T_new_frame_to_init_frame_estimate, frame_pose_estimate.at(new_frame->id));
}

VisualOdometer::PointId
VisualOdometer::generate_point_id()
{
    // NOTE: this is shared among ALL instances of VisualOdometer.
    static std::atomic<PointId> next_id(0);
    return next_id.fetch_add(1);
}
}

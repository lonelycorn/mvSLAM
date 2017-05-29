#pragma once
#include <front-end/data-type.hpp>

namespace mvSLAM
{

// TODO: optimize for memory consumption
class ImagePair
{
public:
    /*
    struct MatchedPoint
    {
        Point3 point;
        size_t vf_idx_in_base;
        size_t vf_idx_in_pair;
    };

    struct MatchedPointEstimate
    {
        Point3Estimate point;
        size_t vf_idx_in_base;
        size_t vf_idx_in_pair;
    };
    */
    struct Params
    {
        /// max distance for a match to be considered as an inlier.
        ScalarType max_match_inlier_distance;
        /// set to true to perform refinement during construction. defaults to false.
        bool refine_structure_in_constructor;
    };

    static Params get_default_params();

    /** Constructor.
     * Will try to reconstruct 3D scene
     * @param refine_structure when set to true, will refine the reconstruction.
     */
    ImagePair(const FrontEndTypes::FramePtr &base_frame_,
              const FrontEndTypes::FramePtr &pair_frame_,
              const Params &params);

    /// Dtor
    ~ImagePair();

    /** Check to see if @p new_frame and @ref base_frame could make a better image pair.
     * @return the quality of new pair, compared with the original pair.
     */
    bool update(const FrontEndTypes::FramePtr &new_frame);

    bool refine();

    // TODO: encapsulate these members
    FrontEndTypes::FramePtr base_frame;
    FrontEndTypes::FramePtr pair_frame;
    bool valid;

    // states that are available after reconstruct()
    uint32_t match_inlier_count; // number of inlier matched descriptors
    uint32_t match_inlier_ssd; // SSD of inlier matched descriptors
    Transformation T_pair_to_base;
    std::vector<Point3> points; // for inliers only
    std::vector<size_t> visual_feature_index_in_base; // for inliers only
    std::vector<size_t> visual_feature_index_in_pair; // for inliers only

    // states that are available after refine()
    ScalarType error; // total error after refinement
    TransformationEstimate T_pair_to_base_estimate;
    std::vector<Point3Estimate> point_estimates; // for inliers only

protected:
    bool reconstruct();

    enum State
    {
        INIT,
        RECONSTRUCTED,
        REFINED,
    };
    State m_state;
    Params m_params;
    VisualFeatureConfig::MatchResultType matches_base_to_pair; // raw matches
    std::vector<size_t> point_index_in_match; // original indexes in @ref matches_base_to_pair of all the inlier points
};

}


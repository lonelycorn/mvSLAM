#pragma once
#include <front-end/data-type.hpp>

namespace mvSLAM
{

// TODO: optimize for memory consumption
class ImagePair
{
public:

    struct MatchedPoint
    {
        Point3 position;
        size_t vf_idx_in_base;
        size_t vf_idx_in_pair;
        /// ctor
        MatchedPoint(Point3 &p, size_t viib, size_t viip):
            position(p), vf_idx_in_base(viib), vf_idx_in_pair(viip)
        {
        }
    };

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

    /** Refine and get the matched point estimate
     *@return true if successfully refined; false otherwise.
     */
    bool refine();

    // TODO: encapsulate these members
    FrontEndTypes::FramePtr base_frame;
    FrontEndTypes::FramePtr pair_frame;
    bool valid;

    // states that are available after reconstruct()
    uint32_t match_inlier_count; // number of inlier matched descriptors
    uint32_t match_inlier_ssd; // SSD of inlier matched descriptors
    Transformation T_pair_to_base;
    std::vector<MatchedPoint> matched_points; // inliers only

    // states that are available after refine()
    ScalarType error; // total error after refinement
    TransformationUncertainty T_pair_to_base_covar;
    std::vector<Point3Uncertainty> matched_points_covar;


protected:
    bool reconstruct(const VisualFeatureConfig::MatchResultType &matches_base_to_pair);

    enum State
    {
        INIT,
        RECONSTRUCTED,
        REFINED,
    };
    State m_state;
    Params m_params;
    VisualFeatureConfig::MatchResultType matches_base_to_pair; // raw matches
};

}


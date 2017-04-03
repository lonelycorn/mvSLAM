#pragma once
#include <base/space.hpp>
#include <front-end/data-type.hpp>

namespace mvSLAM
{

// TODO: optimize for memory consumption
class ImagePair
{
public:
    /** Constructor.
     * Will try to reconstruct 3D scene
     * @param refine_structure when set to true, will refine the reconstruction.
     */
    ImagePair(const FrontEndTypes::FramePtr &base_frame_,
              const FrontEndTypes::FramePtr &pair_frame_,
              bool refine_structure = false);

    /// Dtor
    ~ImagePair();

    /** Check to see if @p new_frame and @ref base_frame could make a better image pair.
     * @return the quality of new pair, compared with the original pair.
     */
    bool update(const FrontEndTypes::FramePtr &new_frame);

    bool refine();

    FrontEndTypes::FramePtr base_frame;
    FrontEndTypes::FramePtr pair_frame;
    bool valid;

    // states that are available after reconstruct()
    uint32_t match_inlier_count; // number of inlier matched descriptors
    uint32_t match_inlier_ssd; // SSD of inlier matched descriptors
    Transformation T_base_to_pair;
    std::vector<Point3> points; // for inliers only

    // states that are available after refine()
    ScalarType error; // total error after refinement
    TransformationEstimate T_base_to_pair_estimate;
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
    VisualFeatureConfig::MatchResultType matches_base_to_pair; // raw matches
    std::vector<size_t> point_indexes; // original indexes in @ref matches_base_to_pair of all the inlier points
};

}


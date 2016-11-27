#pragma once
#include <base/image.hpp>
#include <utility>

namespace mvSLAM
{
class VisualFeature
{
public:
    /** Detect key points and compute descriptors.
     * @param [in] image    rectified image.
     */
    static VisualFeature extract(const ImageGrayscale &image);

    /** Find the match between the two images.
     * @param [in] vf1  visual feature extracted from the first image.
     * @param [in] vf2  visual feature extracted from the second image.
     * @return the matches from 2 to 1.
     */
    static VisualFeatureConfig::MatchResultType
        match_visual_features(const VisualFeature &vf1,
                              const VisualFeature &vf2);

    /** Find the match between the two images.
     * @param [in] image1 a rectified image.
     * @param [in] image2 a rectified image.
     * @return the matches from 2 to 1.
     */
    static VisualFeatureConfig::MatchResultType
        match_images(const ImageGrayscale &image1,
                     const ImageGrayscale &image2);

    /** Find match and remove outliers.
     * @return a std::pair, the first of which contains the matched
     *      visual features from @p vf1, and the second element is
     *      the corresponding visual features from @p vf2.
     */
    static std::pair<VisualFeature, VisualFeature>
        match_and_filter_visual_features(const VisualFeature &vf1,
                                         const VisualFeature &vf2);

    /** Find match and remove outliers.
     * @param [in] image1 a rectified image.
     * @param [in] image2 a rectified image.
     * @return see @ref match_and_filter_visual_features().
     */
    static std::pair<VisualFeature, VisualFeature>
        match_and_filter_images(const ImageGrayscale &image1,
                                const ImageGrayscale &image2);

    /// Ctor
    VisualFeature();
    /// Dtor
    ~VisualFeature();

    VisualFeature(const VisualFeature &) = default;
    VisualFeature(VisualFeature &&) = default;
    VisualFeature &operator=(const VisualFeature &) = default;
    VisualFeature &operator=(VisualFeature &&) = default;

    /// check if contains the same set of keypoints.
    bool equivalent_to(const VisualFeature &other) const;
    
    /// get the number of keypoints.
    size_t size() const;

    /// get all keypoints.
    const VisualFeatureConfig::DetectorResultType &
        get_keypoints() const;
    
    /// get all image points
    std::vector<ImagePoint>
        get_image_points() const;

private:
    VisualFeatureConfig::DetectorResultType m_keypoints;
    VisualFeatureConfig::ExtractorResultType m_descriptors;
};

}

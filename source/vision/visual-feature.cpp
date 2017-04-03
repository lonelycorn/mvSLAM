#include <vision/visual-feature.hpp>
#include <algorithm>
#include <cassert>
#include <unordered_map>

namespace mvSLAM
{
/** Configuration for key point detector */
static VisualFeatureConfig::DetectorContainerType
    _detector = VisualFeatureConfig::DetectorType::create();

/** Configuration for descriptor extractor */
static VisualFeatureConfig::ExtractorContainerType
    _extractor = VisualFeatureConfig::ExtractorType::create();

/** Configuration for descriptor matcher */
static bool cross_check = true;
static VisualFeatureConfig::MatcherType
    _matcher(VisualFeatureConfig::MatcherNormType, cross_check);

// TODO: maybe move this function to common utils?
static void
sort_matches(VisualFeatureConfig::MatchResultType &matches)
{
    auto better_match = [](cv::DMatch &m1, cv::DMatch &m2) -> bool
        {
            return (m1.distance < m2.distance);
        };
    std::sort(matches.begin(), matches.end(), better_match);
}

VisualFeature
VisualFeature::extract(const ImageGrayscale &image)
{
    VisualFeature result;
    _detector->detect(image, result.m_keypoints);
    _extractor->compute(image, result.m_keypoints, result.m_descriptors);
    result.m_image_width = image.cols;
    result.m_image_height = image.rows;
    return result;
}

VisualFeatureConfig::MatchResultType
VisualFeature::match_visual_features(const VisualFeature &vf1,
                                     const VisualFeature &vf2)
{
    VisualFeatureConfig::MatchResultType matches2to1;
    _matcher.match(vf2.m_descriptors,   // queryDescriptors
                   vf1.m_descriptors,   // trainDescriptors
                   matches2to1);        // matches between train and query
    sort_matches(matches2to1);
    return matches2to1;
}

VisualFeatureConfig::MatchResultType
VisualFeature::match_images(const ImageGrayscale &image1,
                            const ImageGrayscale &image2)
{
    VisualFeature vf1 = extract(image1);
    VisualFeature vf2 = extract(image2);
    return match_visual_features(vf1, vf2);
}

std::pair<VisualFeature, VisualFeature>
VisualFeature::match_and_filter_visual_features(const VisualFeature &vf1,
                                               const VisualFeature &vf2)
{
    VisualFeatureConfig::MatchResultType matches2to1 =
        match_visual_features(vf1, vf2);
    
    VisualFeature filtered1;
    VisualFeature filtered2;

    for (auto m : matches2to1)
    {
        filtered1.m_keypoints.push_back(vf1.m_keypoints[m.trainIdx]);
        filtered1.m_descriptors.push_back(vf1.m_descriptors.row(m.trainIdx));
        filtered2.m_keypoints.push_back(vf2.m_keypoints[m.queryIdx]);
        filtered1.m_descriptors.push_back(vf2.m_descriptors.row(m.queryIdx));
    }

    return std::make_pair(filtered1, filtered2);
}

std::pair<VisualFeature, VisualFeature>
VisualFeature::match_and_filter_images(const ImageGrayscale &image1,
                                       const ImageGrayscale &image2)
{
    VisualFeature vf1 = extract(image1);
    VisualFeature vf2 = extract(image2);
    return match_and_filter_visual_features(vf1, vf2);
}

VisualFeature::VisualFeature():
    m_keypoints(), m_descriptors(), m_image_width(-1), m_image_height(-1)
{
}

VisualFeature::~VisualFeature()
{
}

bool
VisualFeature::equivalent_to(const VisualFeature &other) const
{
    if (!valid() || !other.valid())
        return false;

    if (size() != other.size())
        return false;

    std::unordered_map<size_t, size_t> keypoint_count;
    for (auto kp : m_keypoints)
        keypoint_count[kp.hash()] = 1;
    for (auto kp : other.m_keypoints)
    {
        auto it = keypoint_count.find(kp.hash());
        if (it == keypoint_count.end())
            return false;
        ++it->second;
    }
    for (auto item : keypoint_count)
        if (item.second != 2)
            return false;
    return true;
}

size_t
VisualFeature::size() const
{
    return m_keypoints.size();
}

const VisualFeatureConfig::DetectorResultType &
VisualFeature::get_keypoints() const
{
    assert(valid());
    return m_keypoints;
}

std::vector<ImagePoint>
VisualFeature::get_image_points() const
{
    assert(valid());
    std::vector<ImagePoint> result;
    result.reserve(m_keypoints.size());
    for (const auto &kp : m_keypoints)
    {
        result.emplace_back(kp.pt.x, kp.pt.y);
    }
    return result;
}

std::vector<Point2Estimate>
VisualFeature::get_point_estimates() const
{
    assert(valid());
    std::vector<Point2Estimate> result;
    result.reserve(m_keypoints.size());
    for (const auto &kp : m_keypoints)
    {
        // cv::KeyPoint contains
        //  - angle:    orientation of the key point (i.e. major axis); may be unused. 
        //  - class_id: id of object that the key point belongs to; may be unused.
        //  - octave:   level in the image pyramid where the key point is detected. 
        //              Roughly speaking a pixel in the k-th level represents pow(2, k)
        //              pixels in the original image.
        //  - pt:       (u, v) coordinate of the key point.
        //  - response: intensity of the key point (higher is better); may be unused.
        //  - size:     radius of the neighborhood the key point represents.
        // For ORB, the unceritainty of a key point could be empirically modelled as
        // a Gaussian with std dev equals to pow(2, k) * 0.5
        ScalarType stddev = static_cast<ScalarType>(1 << kp.octave) * 0.5;
        Point2 mu{kp.pt.x, kp.pt.y};
        result.emplace_back(mu, sqr(stddev) * Point2Uncertainty::Identity());
    }
    return result;
}

bool
VisualFeature::valid() const
{
    return ((size() > 0) && (m_image_width > 0) && (m_image_height > 0));
}

}

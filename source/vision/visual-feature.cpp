#include <vision/visual-feature.hpp>
#include <algorithm>
#include <cassert>
#include <unordered_map>

namespace mvSLAM
{

constexpr int MAX_FEATURE_COUNT = 500;

/** Configuration for key point detector */
static VisualFeatureConfig::DetectorContainerType
    _detector = VisualFeatureConfig::DetectorType::create(MAX_FEATURE_COUNT);

/** Configuration for descriptor extractor */
static VisualFeatureConfig::ExtractorContainerType
    _extractor = VisualFeatureConfig::ExtractorType::create(MAX_FEATURE_COUNT);

/** Configuration for descriptor matcher */
// NOTE: cannot apply cross check to knn matching
static constexpr bool CROSS_CHECK = false;
static constexpr int NEAREST_NEIGHBOR_COUNT = 2;
static constexpr ScalarType NEAREST_NEIGHBOR_DIST_RATIO = 0.7;
static VisualFeatureConfig::MatcherType
    _matcher(VisualFeatureConfig::MatcherNormType, CROSS_CHECK);


// TODO: maybe move this function to common utils?
static void
sort_matches(VisualFeatureConfig::MatchResultType &matches)
{
    // use of 'auto' in lambda is available since C++14
    auto better_match = [](const cv::DMatch &m1, const cv::DMatch &m2) -> bool
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
                                     const VisualFeature &vf2,
                                     ScalarType max_dist)
{
    std::vector<std::vector<cv::DMatch> > knn_matches2to1;
    _matcher.knnMatch(vf2.m_descriptors,   // queryDescriptors
                      vf1.m_descriptors,   // trainDescriptors
                      knn_matches2to1,// matches between train and query
                      NEAREST_NEIGHBOR_COUNT);
    // lowe ratio test
    auto passed_end = std::partition(knn_matches2to1.begin(), knn_matches2to1.end(),
            [&](const std::vector<cv::DMatch> &knn_m) -> bool
            {
                bool check1 = (knn_m[0].distance < NEAREST_NEIGHBOR_DIST_RATIO * knn_m[1].distance);
                bool check2 = (max_dist < 0) || (knn_m[0].distance <= max_dist);
                return check1 && check2;
            });

    VisualFeatureConfig::MatchResultType matches2to1;
    matches2to1.reserve(passed_end - knn_matches2to1.begin());
    for (auto it = knn_matches2to1.begin(); it != passed_end; ++it)
    {
        matches2to1.push_back(it->at(0));
    }
    sort_matches(matches2to1);
    return matches2to1;
}

/*
VisualFeatureConfig::MatchResultType
VisualFeature::match_images(const ImageGrayscale &image1,
                            const ImageGrayscale &image2)
{
    VisualFeature vf1 = extract(image1);
    VisualFeature vf2 = extract(image2);
    return match_visual_features(vf1, vf2);
}
*/

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

/*
std::pair<VisualFeature, VisualFeature>
VisualFeature::match_and_filter_images(const ImageGrayscale &image1,
                                       const ImageGrayscale &image2)
{
    VisualFeature vf1 = extract(image1);
    VisualFeature vf2 = extract(image2);
    return match_and_filter_visual_features(vf1, vf2);
}
*/

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

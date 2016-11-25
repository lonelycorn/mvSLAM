#include <vision/visual-feature.hpp>
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

VisualFeature
VisualFeature::extract(const ImageGrayscale &image)
{
    VisualFeature result;
    _detector->detect(image, result.m_keypoints);
    _extractor->compute(image, result.m_keypoints, result.m_descriptors);
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
    m_keypoints(), m_descriptors()
{
}

VisualFeature::~VisualFeature()
{
}

bool
VisualFeature::equivalent_to(const VisualFeature &other) const
{
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
    return m_keypoints;
}

/*
std::vector<IdealImagePoint>
VisualFeature::get_ideal_image_points(const CameraIntrinsics &intrinsics)
{
    CameraIntrinsics intrinsics_inv = intrinsics.inverse();
    std::vector<IdealImagePoint> result;
    for (auto kp : m_keypoints)
    {
        Vector3Type v_image;
        v_image << kp.pt.x, 
                   kp.pt.y, 
                   (ScalarType) 1.0;
        Vector3Type v_ideal = intrinsics_inv * v;
        result.push_back(IdealImagePoint(v_ideal[0], v_idea[1]));
    }
    return result;
}
*/

std::vector<ImagePoint>
VisualFeature::get_image_points() const
{
    std::vector<ImagePoint> result;
    cv::KeyPoint::convert(m_keypoints, result);
    return result;
}
}

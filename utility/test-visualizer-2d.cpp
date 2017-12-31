#include <iostream>

#include <system-config.hpp>
#include <base/error.hpp>
#include <base/image.hpp>
#include <vision/visual-feature.hpp>
#include <visualization/data-type.hpp>
#include <visualization/visualizer-2d.hpp>

int main()
{
    const std::string image_path("../data/tsukuba/");
    const mvSLAM::ScalarType match_max_dist = 10;

    mvSLAM::KeyPointImagePair kpip;

    auto &base = kpip.base;
    base.id = 0;
    base.image = mvSLAM::load_image_grayscale(image_path + "1.jpg");

    auto &pair = kpip.pair;
    pair.id = 1;
    pair.image = mvSLAM::load_image_grayscale(image_path + "2.jpg");

    mvSLAM::VisualFeature vf_base = mvSLAM::VisualFeature::extract(base.image);
    mvSLAM::VisualFeature vf_pair = mvSLAM::VisualFeature::extract(pair.image);

    auto matches_pair_to_base = mvSLAM::VisualFeature::match_visual_features(vf_base, vf_pair);

    {
        const auto &kp = vf_base.get_image_points();
        base.keypoints.reserve(kp.size());
        for (size_t i = 0; i < kp.size(); ++i)
        {
            const auto &p1 = kp[i];
            mvSLAM::Point2 p2;
            p2 << p1.x, p1.y;
            base.keypoints.emplace(i, p2);
        }
    }
    {
        const auto &kp = vf_pair.get_image_points();
        pair.keypoints.reserve(kp.size());
        for (size_t i = 0; i < kp.size(); ++i)
        {
            const auto &p1 = kp[i];
            mvSLAM::Point2 p2;
            p2 << p1.x, p1.y;
            pair.keypoints.emplace(i, p2);
        }
    }

    kpip.raw_matches.reserve(matches_pair_to_base.size());
    kpip.inlier_matches.reserve(matches_pair_to_base.size());
    for (const auto &m : matches_pair_to_base)
    {
        if (m.distance < match_max_dist) // inlier
        {
            kpip.inlier_matches.emplace_back(m.trainIdx, m.queryIdx);
        }
        kpip.raw_matches.emplace_back(m.trainIdx, m.queryIdx);
    }

    const auto params = mvSLAM::Visualizer2d::get_default_params();
    mvSLAM::Visualizer2d viewer("whooja", params);

    viewer.set_key_frame(base);

    viewer.set_matched_image_pair(kpip);

    mvSLAM::sleep_ms(5000);

    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}

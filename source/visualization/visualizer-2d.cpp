#include <base/debug.hpp>
#include <base/parameter-manager.hpp>
#include <os/mutex.hpp>
#include <os/event.hpp>
#include <visualization/visualizer-2d.hpp>

#include <opencv2/highgui.hpp>

#include <cassert>
#include <atomic>
#include <thread>
#include <deque>

namespace mvSLAM
{
static const std::string module_name("visualizer2d");
static Logger logger("[visualizer2d]", true);

// NOTE: openCV uses BGR
static const cv::Scalar color_blue(0xff, 0x00, 0x00);
static const cv::Scalar color_green(0x00, 0xff, 0x00);
static const cv::Scalar color_red(0x00, 0x00, 0xff);

/// Data container for @ref Visualizer2d
struct Visualizer2dImpl
{
    enum class RedrawObject
    {
        KeyFrame,
        MatchedImagePair,
        Count,
    };

    const std::string name;
    const Visualizer2d::Params params;

    Mutex mutex; // protects everything down below
    Event event;
    bool viewer_thread_should_exit;
    std::thread viewer_thread;
    std::vector<cv::Mat> canvas;
    std::deque<RedrawObject> redraw_object_queue;
    VisualizationTypes::KeyFrame key_frame;
    VisualizationTypes::MatchedImagePair matched_image_pair;

    Visualizer2dImpl(const std::string &n, const Visualizer2d::Params &p):
        name(n),
        params(p),
        mutex(),
        event(),
        viewer_thread_should_exit(false),
        viewer_thread(),
        canvas(static_cast<size_t>(RedrawObject::Count)),
        redraw_object_queue(),
        key_frame(),
        matched_image_pair()
    {
    }
    ~Visualizer2dImpl() = default;
};

/**
 * @param canvas cv::Mat with the same dimension as @p kf; type is 8UC3
 */
static void
draw_key_frame(cv::Mat &canvas,
               const VisualizationTypes::KeyFrame &kf,
               const Visualizer2d::Params &params)
{
    int rows = kf.image.rows;
    int cols = kf.image.cols;
    assert((rows > 0) && (cols > 0));

    if (canvas.empty())
    {
        canvas.create(rows, cols, CV_8UC3); // 3 channel
    }

    cv::cvtColor(kf.image, canvas, CV_GRAY2BGR); // convert grayscale to color

    for (const auto &id_point_pair : kf.keypoints)
    {
        const auto &pt = id_point_pair.second;
        cv::circle(canvas,  // image
                   cv::Point(pt[0], pt[1]), // center
                   params.point_size_pixel, // radius
                   color_red, // color
                   1, // thickness
                   8, // lineType
                   0); // shift
    }
}

// NOTE: cv::Mat is just a header
static void
draw_matched_image_pair(cv::Mat &canvas,
                        const VisualizationTypes::MatchedImagePair &mip,
                        const Visualizer2d::Params &params)
{
    assert(mip.base.image.rows == mip.pair.image.rows);
    assert(mip.base.image.cols == mip.pair.image.cols);
    assert((mip.base.image.rows > 0) && (mip.base.image.cols > 0));

    int rows = mip.base.image.rows;
    int cols = mip.base.image.cols;

    if (canvas.empty())
    {
        canvas.create(2 * rows, cols, CV_8UC3); // base & pair, 3 channel
    }

    // draw base on the top
    cv::Mat top = canvas.rowRange(cv::Range(0, rows));
    draw_key_frame(top, mip.base, params);

    // draw pair on the bottom
    cv::Mat bottom = canvas.rowRange(cv::Range(rows, rows * 2));
    draw_key_frame(bottom, mip.pair, params);

    // draw lines connecting raw matches
    for (const auto &p : mip.raw_matches)
    {
        auto base_id = p.first;
        auto pair_id = p.second;

        const auto &base_pt = mip.base.keypoints.at(base_id);
        const auto &pair_pt = mip.pair.keypoints.at(pair_id);

        cv::line(canvas,
                 cv::Point(base_pt[0], base_pt[1]),
                 cv::Point(pair_pt[0], pair_pt[1] + rows),
                 color_blue,
                 params.line_width_pixel,
                 8, // lineType
                 0); // shift
    }

    // draw lines connecting inlier matches
    for (const auto &p : mip.inlier_matches)
    {
        auto base_id = p.first;
        auto pair_id = p.second;

        const auto &base_pt = mip.base.keypoints.at(base_id);
        const auto &pair_pt = mip.pair.keypoints.at(pair_id);

        cv::line(canvas,
                 cv::Point(base_pt[0], base_pt[1]),
                 cv::Point(pair_pt[0], pair_pt[1] + rows),
                 color_green,
                 params.line_width_pixel,
                 8, // lineType
                 0); // shift
    }
}

static void
run_viewer_thread(Visualizer2dImpl *v)
{
    assert(v);
    logger.info("thread start");
    while (true)
    {
        Lock lock(v->mutex);
        if (v->viewer_thread_should_exit)
        {
            break;
        }

        // NOTE: spurious wakeups may occur and usually conditional variables are used
        // with a boolean predicative. here we just want to check the queue periodically
        // without busy-waiting, so the return value doesn't matter
        v->event.wait_timeout(v->mutex, v->params.view_update_interval_ms);

        while (!v->redraw_object_queue.empty())
        {
            switch (v->redraw_object_queue.front())
            {
            case Visualizer2dImpl::RedrawObject::KeyFrame:
                {
                    // NOTE: must use index to access the content because it may change
                    size_t idx = static_cast<size_t>(Visualizer2dImpl::RedrawObject::KeyFrame);
                    draw_key_frame(v->canvas[idx], v->key_frame, v->params);
                    cv::imshow("KeyFrame", v->canvas[idx]);
                    break;
                }
            case Visualizer2dImpl::RedrawObject::MatchedImagePair:
                {
                    size_t idx = static_cast<size_t>(Visualizer2dImpl::RedrawObject::MatchedImagePair);
                    draw_matched_image_pair(v->canvas[idx], v->matched_image_pair, v->params);
                    cv::imshow("ImagePair", v->canvas[idx]);
                    break;
                }
            default:
                assert(false); // should not happen
            }
            v->redraw_object_queue.pop_front();
        }

        cv::waitKey(1); // wait 1 ms
    }
    logger.info("thread stop");
}

Visualizer2d::Params
Visualizer2d::get_default_params()
{
    Params p;

    p.view_update_interval_ms = ParameterManager::get_value(
            module_name, "view_update_interval_ms", 200);

    p.view_update_immediately = ParameterManager::get_value(
            module_name, "view_update_immediately", false);

    p.point_size_pixel = ParameterManager::get_value(
            module_name, "point_size_pixel", 4);

    p.line_width_pixel = ParameterManager::get_value(
            module_name, "line_width_pixel", 2);

    return p;
}

Visualizer2d::Visualizer2d(const std::string &name, const Params &params):
    m_impl(new Visualizer2dImpl(name, params))
{
    logger.info("constructor, ", m_impl->name);

    Lock lock(m_impl->mutex);

    m_impl->viewer_thread = std::thread(run_viewer_thread, m_impl);
}

Visualizer2d::~Visualizer2d()
{
    logger.info("desctructor, ", m_impl->name);

    {
        Lock lock(m_impl->mutex);
        m_impl->viewer_thread_should_exit = true;
    }
    m_impl->viewer_thread.join(); // blocking-wait for thread to finish

    // m_impl's dtor is called here
    delete m_impl;
    m_impl = nullptr;
}

void
Visualizer2d::set_key_frame(const VisualizationTypes::KeyFrame &kf)
{
    Lock lock(m_impl->mutex);

    m_impl->redraw_object_queue.push_back(Visualizer2dImpl::RedrawObject::KeyFrame);
    m_impl->key_frame = kf;

    if (m_impl->params.view_update_immediately)
    {
        m_impl->event.trigger_all();
    }
}

void
Visualizer2d::set_matched_image_pair(const VisualizationTypes::MatchedImagePair &mip)
{
    Lock lock(m_impl->mutex);

    m_impl->redraw_object_queue.push_back(Visualizer2dImpl::RedrawObject::MatchedImagePair);
    m_impl->matched_image_pair = mip;

    if (m_impl->params.view_update_immediately)
    {
        m_impl->event.trigger_all();
    }
}


}

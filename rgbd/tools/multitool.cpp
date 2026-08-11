#include "rgbd/client.h"
#include "rgbd/types.h"
#include "rgbd/view.h"

#include <algorithm>
#include <geolib/datatypes.h>
#include <iostream>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>

#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

namespace
{

struct MultitoolState
{
    bool paused = false;
    std::string mode;
    int image_width = 0;
    int image_height = 0;
    std::vector<cv::Vec2i> mouse_points;
    cv::Vec2i mouse_pos;
};

// ----------------------------------------------------------------------------------------------------

void callBackFunc(int event, int x, int y, int /*flags*/, void* userdata)
{
    auto* state = static_cast<MultitoolState*>(userdata);
    x = x % state->image_width;
    state->mouse_pos = cv::Vec2i(x, y);

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        state->mouse_points.push_back(state->mouse_pos);
    }
}

} // namespace

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "rgbd_multitool", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto logger = node->get_logger();

    std::unique_ptr<rgbd::Client> client(nullptr);
    MultitoolState state;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // - - - - - - - - - - - -

    // Parse command line arguments

    if (argc < 3)
    {
        std::cout << "Usage:" << '\n' << '\n' << "    multitool --rgbd RGBD_TOPIC" << '\n' << '\n';
        return 1;
    }

    for (int i = 1; i < argc; i += 2)
    {
        std::string const opt = argv[i];
        std::string const arg = argv[i + 1];

        if (opt == "--rgbd")
        {
            client = std::make_unique<rgbd::Client>();
            client->initialize(node->get_node_topics_interface()->resolve_topic_name(arg));
        }
        else
        {
            std::cout << "Unknown option: '" << opt << "'." << '\n';
            return 1;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // - - - - - - - - - - - -

    std::cout << "Keys:" << '\n'
              << '\n'
              << "    spacebar - Pause" << '\n'
              << "    m        - Measure" << '\n'
              << "    q        - Quit" << '\n'
              << '\n';

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // - - - - - - - - - - - -

    const std::string window_name = "RGBD";
    // Create a window
    cv::namedWindow(window_name, 1);

    // set the callback function for any mouse event
    cv::setMouseCallback(window_name, callBackFunc, &state);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // - - - - - - - - - - - -

    float const max_view_distance = 10;

    rgbd::ImagePtr image;

    double rate = 30.0;
    if (!node->has_parameter("rate"))
    {
        node->declare_parameter<double>("rate", rate);
    }
    node->get_parameter("rate", rate);

    rclcpp::Rate r(rate);
    while (rclcpp::ok())
    {
        if (!state.paused && client)
        {
            rgbd::ImagePtr const image_tmp = client->nextImage();
            if (image_tmp)
                image = image_tmp;
        }

        cv::Mat canvas;
        state.image_width = 0;
        state.image_height = 0;

        if (image)
        {
            const cv::Mat& rgb = image->getRGBImage();
            const cv::Mat& depth = image->getDepthImage();

            if (depth.data)
            {
                cv::Mat depth_canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(50, 0, 0));
                for (int y = 0; y < depth.rows; ++y)
                {
                    for (int x = 0; x < depth.cols; ++x)
                    {
                        float const d = depth.at<float>(y, x);
                        if (d > 0 && d == d)
                        {
                            unsigned char const v = static_cast<unsigned char>(
                                std::min<float>(max_view_distance, d / max_view_distance) * 255);
                            depth_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(v, v, v);
                        }
                    }
                }

                if (rgb.data)
                {
                    state.image_width = std::min(rgb.cols, depth.cols);

                    int const rgb_height = state.image_width * rgb.rows / rgb.cols;
                    int const depth_height = state.image_width * depth.rows / depth.cols;

                    state.image_height = std::max(rgb_height, depth_height);

                    canvas = cv::Mat(state.image_height, state.image_width * 2, CV_8UC3, cv::Scalar(50, 50, 50));

                    cv::Mat rgb_roi = canvas(cv::Rect(cv::Point(0, 0), cv::Size(state.image_width, rgb_height)));
                    cv::Mat depth_roi =
                        canvas(cv::Rect(cv::Point(state.image_width, 0), cv::Size(state.image_width, depth_height)));

                    cv::resize(rgb, rgb_roi, cv::Size(state.image_width, rgb_height));
                    cv::resize(depth_canvas, depth_roi, cv::Size(state.image_width, depth_height));
                }
                else
                {
                    canvas = depth_canvas;
                }
            }
            else if (rgb.data)
            {
                canvas = rgb;
            }
        }

        if (!canvas.data)
        {
            canvas = cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 50, 50));
            cv::line(canvas, cv::Point(0, 0), cv::Point(640, 480), cv::Scalar(255, 255, 255), 5);
            cv::line(canvas, cv::Point(0, 480), cv::Point(640, 0), cv::Scalar(255, 255, 255), 5);
        }

        if (state.image_width == 0 || state.image_height == 0)
        {
            state.image_width = canvas.cols;
            state.image_height = canvas.rows;
        }

        // Show mouse cursor(s)
        cv::circle(canvas, state.mouse_pos, 5, cv::Scalar(255, 0, 0), 1);
        if (canvas.cols > state.image_width)
            cv::circle(canvas, state.mouse_pos + cv::Vec2i(state.image_width, 0), 5, cv::Scalar(255, 0, 0), 1);

        cv::putText(
            canvas, state.mode, cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

        if (state.paused)
            cv::putText(canvas,
                        "PAUSED",
                        cv::Point(10, canvas.rows - 25),
                        cv::FONT_HERSHEY_COMPLEX_SMALL,
                        1,
                        cv::Scalar(255, 255, 255),
                        1);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // - - - - - - - - - - - - -

        for (const auto& mouse_point : state.mouse_points)
        {
            cv::circle(canvas, mouse_point, 5, cv::Scalar(0, 0, 255), 2);
            if (canvas.cols > state.image_width)
                cv::circle(canvas, mouse_point + cv::Vec2i(state.image_width, 0), 5, cv::Scalar(0, 0, 255), 2);
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // - - - - - - - - - - - - -
        if (state.mode == "DONE")
        {
            break;
        }
        if (state.mode == "MEASURE")
        {
            if (state.mouse_points.size() == 2)
            {
                rgbd::View const view(*image, 640);

                geo::Vector3 p1;
                geo::Vector3 p2;

                if (view.getPoint3D(state.mouse_points[0][0], state.mouse_points[0][1], p1) &&
                    view.getPoint3D(state.mouse_points[1][0], state.mouse_points[1][1], p2))
                {
                    std::cout << (p1 - p2).length() << " m" << '\n';
                }

                state.mouse_points.clear();
            }
        }
        else
        {
            state.mouse_points.clear();
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // - - - - - - - - - - - - -

        cv::imshow(window_name, canvas);
        int const i_key = cv::waitKey(3);
        if (i_key >= 0)
        {
            char const key = static_cast<char>(i_key);

            switch (key)
            {
            case ' ': state.paused = !state.paused; break;
            case 'm': state.mode = state.mode == "MEASURE" ? "" : "MEASURE"; break;
            case 'q': state.mode = "DONE"; break;
            default: state.mode = ""; break;
            }
        }

        r.sleep();
    }

    RCLCPP_INFO(logger, "Shutting down");
    rclcpp::shutdown();
    return 0;
}

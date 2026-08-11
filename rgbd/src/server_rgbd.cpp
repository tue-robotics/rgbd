#include "rgbd/server_rgbd.h"

#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <rgbd_interfaces/msg/detail/rgbd__struct.hpp>
#include <rgbd_interfaces/srv/detail/get_rgbd__struct.hpp>
#include <string>
#include <tue/serialization/conversions.h>

#include <sstream>
#include <tue/serialization/output_archive.h>

#include "rgbd/image.h"
#include "rgbd/serialization.h"

namespace rgbd
{

const int ServerRGBD::MESSAGE_VERSION = 4;

ServerRGBD::ServerRGBD(const rclcpp::Node::SharedPtr& node) :
    node_(node ? node : rclcpp::Node::make_shared("rgbd_server_rgbd"))
{
}

ServerRGBD::~ServerRGBD()
{
    stop_service_thread_ = true;
    if (service_thread_.joinable())
    {
        service_thread_.join();
    }
    if (executor_)
    {
        executor_->remove_node(node_);
    }
}

void ServerRGBD::initialize(const std::string& name,
                            RGBStorageType rgb_type,
                            DepthStorageType depth_type,
                            float service_freq)
{
    pub_image_ = node_->create_publisher<rgbd_interfaces::msg::RGBD>(name, 1);
    rgb_type_ = rgb_type;
    depth_type_ = depth_type;

    service_server_ = node_->create_service<rgbd_interfaces::srv::GetRGBD>(
        name,
        [this](const std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Request>& req,
               const std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Response>& resp) { serviceCallback(req, resp); });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    service_thread_ = std::thread(&ServerRGBD::serviceThreadFunc, this, service_freq);
}

void ServerRGBD::send(const Image& image)
{
    {
        std::unique_lock<std::mutex> const ul(image_mutex_);
        image_ = image.clone();
    }

    if (pub_image_->get_subscription_count() == 0)
    {
        return;
    }

    auto msg = std::make_shared<rgbd_interfaces::msg::RGBD>();
    msg->version = MESSAGE_VERSION;

    std::stringstream stream;
    std::stringstream stream2;
    tue::serialization::OutputArchive a(stream);
    serialize(image, a, rgb_type_, depth_type_);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::zstd_compressor(boost::iostreams::zstd::best_speed));
    in.push(stream);
    boost::iostreams::copy(in, stream2);
    tue::serialization::convert(stream2, msg->rgb);

    pub_image_->publish(*msg);
}

void ServerRGBD::serviceCallback(const std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Request>& REQ,
                                 const std::shared_ptr<rgbd_interfaces::srv::GetRGBD::Response>& resp)
{
    rgbd::Image image;
    {
        std::unique_lock<std::mutex> const ul(image_mutex_);
        image = image_.clone();
    }

    if (REQ->compression != rgbd_interfaces::srv::GetRGBD::Request::JPEG &&
        REQ->compression != rgbd_interfaces::srv::GetRGBD::Request::PNG)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ServerRGBD"),
                     "Invalid compression, only JPEG and PNG are supported (see ENUM in srv definition)");
        return;
    }

    cv::Mat resized_rgb;
    cv::Mat resized_depth;

    const double ratio_rgb = static_cast<double>(REQ->width) / static_cast<double>(image.getRGBImage().cols);
    const double ratio_depth = static_cast<double>(REQ->width) / static_cast<double>(image.getDepthImage().cols);

    cv::resize(
        image.getRGBImage(), resized_rgb, cv::Size(REQ->width, static_cast<int>(image.getRGBImage().rows * ratio_rgb)));
    cv::resize(image.getDepthImage(),
               resized_depth,
               cv::Size(REQ->width, static_cast<int>(image.getDepthImage().rows * ratio_depth)));

    const std::string compression_str =
        REQ->compression == rgbd_interfaces::srv::GetRGBD::Request::JPEG ? ".jpeg" : ".png";
    if (!cv::imencode(compression_str, resized_rgb, resp->rgb_data) ||
        !cv::imencode(compression_str, resized_depth, resp->depth_data))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("ServerRGBD"), "cv::imencode with compression_str %s failed!", compression_str.c_str());
    }
}

void ServerRGBD::serviceThreadFunc(float frequency)
{
    rclcpp::Rate r(frequency);
    while (rclcpp::ok() && !stop_service_thread_)
    {
        executor_->spin_some();
        r.sleep();
    }
}

} // namespace rgbd

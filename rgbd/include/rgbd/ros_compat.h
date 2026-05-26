#ifndef RGBD_ROS_COMPAT_H_
#define RGBD_ROS_COMPAT_H_

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>

namespace ros
{

namespace init_options
{
constexpr uint32_t AnonymousName = 1U;
} // namespace init_options

inline void init(int& argc, char**& argv, const std::string& /*name*/, uint32_t /*options*/ = 0)
{
    rclcpp::init(argc, argv);
}

inline void start() {}

inline bool ok()
{
    return rclcpp::ok();
}

class Time
{
public:
    Time() = default;
    explicit Time(double sec) : sec_(sec) {}

    static Time now() { return Time(rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds()); }

    double toSec() const { return sec_; }

private:
    double sec_ = 0;
};

inline std::ostream& operator<<(std::ostream& out, const Time& time)
{
    out << time.toSec();
    return out;
}

class WallDuration
{
public:
    explicit WallDuration(double sec) : duration_(std::chrono::duration<double>(sec)) {}

private:
    friend class WallTime;
    std::chrono::duration<double> duration_;
};

class WallTime
{
public:
    static WallTime now() { return WallTime(std::chrono::steady_clock::now()); }

    friend bool operator>=(const WallTime& lhs, const WallTime& rhs) { return lhs.time_point_ >= rhs.time_point_; }

    friend WallTime operator+(const WallTime& lhs, const WallDuration& rhs)
    {
        return WallTime(lhs.time_point_ +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(rhs.duration_));
    }

private:
    explicit WallTime(std::chrono::steady_clock::time_point time_point) : time_point_(time_point) {}

    std::chrono::steady_clock::time_point time_point_;
};

class Rate
{
public:
    explicit Rate(double hz) : rate_(hz) {}

    void sleep() { rate_.sleep(); }

private:
    rclcpp::Rate rate_;
};

class NodeHandle
{
public:
    explicit NodeHandle(const std::string& ns = "~")
    {
        std::string node_name = "rgbd_tool";
        if (ns != "~" && !ns.empty())
        {
            node_name = ns;
        }
        node_ = std::make_shared<rclcpp::Node>(
            node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    }

    template <typename T> bool getParam(const std::string& name, T& value)
    {
        if (!node_->has_parameter(name))
        {
            node_->declare_parameter<T>(name, value);
        }
        return node_->get_parameter(name, value);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
};

namespace names
{
inline std::string resolve(const std::string& name)
{
    return name;
}
} // namespace names

namespace master
{
inline bool check()
{
    return rclcpp::ok();
}
} // namespace master

} // namespace ros

#define ROS_INFO(...)         RCLCPP_INFO(rclcpp::get_logger("rgbd"), __VA_ARGS__)
#define ROS_WARN(...)         RCLCPP_WARN(rclcpp::get_logger("rgbd"), __VA_ARGS__)
#define ROS_ERROR(...)        RCLCPP_ERROR(rclcpp::get_logger("rgbd"), __VA_ARGS__)
#define ROS_FATAL(...)        RCLCPP_FATAL(rclcpp::get_logger("rgbd"), __VA_ARGS__)
#define ROS_INFO_STREAM(msg)  RCLCPP_INFO_STREAM(rclcpp::get_logger("rgbd"), msg)
#define ROS_WARN_STREAM(msg)  RCLCPP_WARN_STREAM(rclcpp::get_logger("rgbd"), msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(rclcpp::get_logger("rgbd"), msg)

#endif // RGBD_ROS_COMPAT_H_

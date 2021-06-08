#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>


namespace rgbd
{

class Plus : public nodelet::Nodelet
{
public:
  Plus()
  : value_(0)
  {}

private:
  virtual void onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    private_nh.getParam("value", value_);
    pub = private_nh.advertise<std_msgs::Float64>("out", 10);
    sub = private_nh.subscribe("in", 10, &Plus::callback, this);
  }

  void callback(const std_msgs::Float64::ConstPtr& input)
  {
    std_msgs::Float64Ptr output(new std_msgs::Float64());
    output->data = input->data + value_;
    NODELET_DEBUG("Adding %f to get %f", value_, output->data);
    pub.publish(output);
  }

  ros::Publisher pub;
  ros::Subscriber sub;
  double value_;
};

PLUGINLIB_EXPORT_CLASS(test_nodelet::Plus, nodelet::Nodelet)
}

#include <rgbd/View.h>
#include <rgbd/Client.h>
#include <opencv2/highgui/highgui.hpp>

#include <geolib/datatypes.h>
#include <geolib/ros/msg_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/service_client.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <memory>

#include <rgbd/Project2DTo3D.h>

// ----------------------------------------------------------------------------------------------------

std::shared_ptr<rgbd::Image> g_last_img;
bool srvGet3dPointFromRoi(rgbd::Project2DTo3D::Request& req, rgbd::Project2DTo3D::Response& res)
{
  if (!g_last_img) {
    ROS_ERROR("I did not receive a rgbd image yet ...");
    return false;
  }
  const sensor_msgs::RegionOfInterest& roi = req.roi;

  const cv::Mat& depth = g_last_img->getDepthImage();

  cv::Rect roi_rgb(roi.x_offset, roi.y_offset, roi.width, roi.height);
  float rgb_depth_width_ratio = (float)(depth.cols) / g_last_img->getRGBImage().cols;
  cv::Rect roi_depth(rgb_depth_width_ratio * roi_rgb.tl(), rgb_depth_width_ratio * roi_rgb.br());
  cv::Point roi_depth_center = 0.5 * (roi_depth.tl() + roi_depth.br());

  cv::Rect roi_depth_capped(cv::Point(std::max(0, roi_depth.x),
                                      std::max(0, roi_depth.y)),
                            cv::Point(std::min(depth.cols - 1, roi_depth.br().x),
                                      std::min(depth.rows - 1, roi_depth.br().y)));

  cv::Mat depth_roi_capped = depth(roi_depth_capped);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Get median depth of ROI

  std::vector<float> depths;
  for(unsigned int j = 0; j < depth_roi_capped.cols * depth_roi_capped.rows; ++j)
  {
      float d = depth_roi_capped.at<float>(j);
      if (d > 0 && d == d)
          depths.push_back(d);
  }

  if (depths.empty())
  {
    ROS_ERROR("All depths within ROI are invalid!");
    return false;
  }

  std::sort(depths.begin(), depths.end());
  float median_depth = depths[depths.size() / 2];

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Determine roi location

  rgbd::View view(*g_last_img, g_last_img->getDepthImage().cols);
  geo::Vec3 pos = view.getRasterizer().project2Dto3D(roi_depth_center.x, roi_depth_center.y) * median_depth;

  ROS_INFO("Pose (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);

  res.point.header.frame_id = g_last_img->getFrameId();
  res.point.header.stamp = ros::Time(g_last_img->getTimestamp());
  geo::convert(pos, res.point.point);
  return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_3d_point_from_image_roi");

  // Listener
  rgbd::Client client;
  client.intialize("rgbd");

  // srv
  ros::NodeHandle nh;
  ros::ServiceServer srv_project_2d_to_3d = nh.advertiseService("project_2d_to_3d", srvGet3dPointFromRoi);

  ros::Rate r(30);
  while (ros::ok())
  {
    ros::spinOnce();
    rgbd::Image image;
    if (client.nextImage(image))
    {
      if (image.getDepthImage().data)
      {
        g_last_img = std::shared_ptr<rgbd::Image>(new rgbd::Image(image));
        ROS_INFO("Got msg...");
      }
    }
  }
}

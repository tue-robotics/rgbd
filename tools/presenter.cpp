#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

struct ChannelView
{
    ChannelView(const std::string& topic, double rx_, double ry_, double rw_)
        : rx(rx_), ry(ry_), rwidth(rw_)
    {
        client.reset(new rgbd::Client());
        client->intialize(topic);
    }

    boost::shared_ptr<rgbd::Client> client;
    double rx;
    double ry;
    double rwidth;
};

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_viewer");

    std::vector<ChannelView> views;

    views.push_back(ChannelView("/amigo/ed/kinect/viz/update_request", 0, 0, 0.7));
    views.push_back(ChannelView("/amigo/ed/kinect/viz/sensor_normals", 0.71, 0, 0.28));
    views.push_back(ChannelView("/amigo/ed/kinect/viz/model_normals", 0.71, 0.26, 0.28));

    cv::namedWindow("Presenter", CV_WINDOW_NORMAL);
    cv::setWindowProperty("Presenter", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::Mat canvas(600, 1024, CV_8UC3, cv::Scalar(0, 0, 0));

    while (ros::ok())
    {
        for(std::vector<ChannelView>::iterator it = views.begin(); it != views.end(); ++it)
        {
            ChannelView& view = *it;

            rgbd::Image image;
            if (!view.client->nextImage(image))
                continue;

            cv::Mat rgb = image.getRGBImage();
            if (!rgb.data)
                continue;

            // Show rgb image
            int x = view.rx * canvas.cols;
            int y = view.ry * canvas.cols; // .cols is NOT A TYPO!
            int width = view.rwidth * canvas.cols;
            int height = ((double)rgb.rows / rgb.cols) * width;

            cv::Mat rgb_resized;
            cv::resize(rgb, rgb_resized, cv::Size(width, height));

            rgb_resized.copyTo(canvas(cv::Rect(x, y, width, height)));
        }

        cv::imshow("Presenter", canvas);
        char key = cv::waitKey(10);
        if (key == 'q' || (int)key == 27) // q or escape
            return 0;

    }

    return 0;
}

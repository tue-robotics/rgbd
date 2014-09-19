#include <string>
#include <iostream>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <opencv2/core/core.hpp>

class Server
{

public:

    Server() : width_(0), height_(0)
    {
    }

    ~Server()
    {
        // Clean up after ourselves
        boost::interprocess::shared_memory_object::remove("MySharedMemory");
    }

    void sendImage(const cv::Mat& image)
    {
        using namespace boost::interprocess;

        if (width_ == 0)
        {
            // First time
            // Make sure possibly existing memory with same name is removed
            boost::interprocess::shared_memory_object::remove("MySharedMemory");

            //Create a shared memory object.
            shm_ = shared_memory_object(create_only, "MySharedMemory", read_write);

            //Set size
            shm_.truncate(sizeof(interprocess_mutex) + image.cols * image.rows * 3);

            // Map two regions: one of the mutex, on for the image
            mem_mutex_ = mapped_region(shm_, read_write, 0, sizeof(interprocess_mutex));

            mem_image_ = mapped_region(shm_, read_write, sizeof(interprocess_mutex));

            width_ = image.cols;
            height_ = image.rows;
        }

        interprocess_mutex* mutex = static_cast<interprocess_mutex*>(mem_mutex_.get_address());
        std::cout << mutex << std::endl;

        {
            std::cout << "Going to lock..." << std::endl;
//            scoped_lock<interprocess_mutex> lock(*mutex);
            std::cout << ".... Locked." << std::endl;

            memcpy(mem_image_.get_address(), image.data, image.cols * image.rows * 3);
        }
        std::cout << "Unlocked" << std::endl;

    }

private:

    int width_, height_;

    boost::interprocess::shared_memory_object shm_;

    boost::interprocess::mapped_region mem_mutex_;

    boost::interprocess::mapped_region mem_image_;

};

int main(int argc, char *argv[])
{
    Server server;

    int x = 0;
    while(true)
    {
        cv::Mat image(480, 640, CV_8UC3, cv::Scalar(100, 100, 100));
        cv::line(image, cv::Point(x, 0), cv::Point(x, image.rows - 1), cv::Scalar(255, 0, 0));

        server.sendImage(image);
        std::cout << "Sent image: " << x << std::endl;

        x = (x + 10) % image.cols;

        usleep(10000);
    }

    return 0;
}


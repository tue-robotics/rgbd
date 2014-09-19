#include <string>
#include <iostream>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Client
{

public:

    Client() : width_(0), height_(0)
    {
    }

    ~Client()
    {
        boost::interprocess::shared_memory_object::remove("MySharedMemory");
    }

    bool nextImage(cv::Mat& image)
    {
        using namespace boost::interprocess;

        if (width_ == 0)
        {           
            // Open already created shared memory object.
            shm_ = shared_memory_object(open_only, "MySharedMemory", read_write);

            offset_t size;
            shm_.get_size(size);
            std::cout << "Shared memory size = " << size << " bytes" << std::endl;

            // Map two regions: one of the mutex, on for the image
            mem_mutex_ = mapped_region(shm_, read_write, 0, sizeof(interprocess_mutex));
            mem_image_ = mapped_region(shm_, read_write, sizeof(interprocess_mutex));

            width_ = 640;
            height_ = 480;
        }

        interprocess_mutex* mutex = static_cast<interprocess_mutex*>(mem_mutex_.get_address());
        std::cout << mutex << std::endl;

        image = cv::Mat(height_, width_, CV_8UC3);

        {
            std::cout << "Going to lock..." << std::endl;
//            scoped_lock<interprocess_mutex> lock(*mutex);
            std::cout << ".... Locked." << std::endl;

            memcpy(image.data, mem_image_.get_address(), image.cols * image.rows * 3);
        }
        std::cout << "Unlocked" << std::endl;

        return true;
    }

private:

    int width_, height_;

    boost::interprocess::shared_memory_object shm_;

    boost::interprocess::mapped_region mem_mutex_;

    boost::interprocess::mapped_region mem_image_;
};

int main(int argc, char *argv[])
{
    Client client;

    while(true)
    {
        cv::Mat image;
        if (client.nextImage(image))
        {
            cv::imshow("Image", image);
            cv::waitKey(3);
        }

        usleep(10000);
    }

    return 0;
}


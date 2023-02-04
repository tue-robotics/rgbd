#include <boost/interprocess/shared_memory_object.hpp>

#include <gtest/gtest.h>

#include <rgbd/client.h>
#include <rgbd/image.h>

#include <ros/duration.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/rate.h>
#include <ros/time.h>


namespace ipc = boost::interprocess;

class Connection : public testing::Test
{
protected:
    void SetUp() override
    {
        client.initialize(ros::names::resolve(test_server_name));
        ros::Time end_init = ros::Time::now() + ros::Duration(2);
        ros::Rate r_init(10);
        while(ros::Time::now() < end_init)
        {
            if (client.nextImage())
            {
                ros::Duration(0.5).sleep(); // Make sure there will be a new image when going to the actual test phase
                break;
            }
            r_init.sleep();
        }
    }

    const std::string test_server_name = "rgbd";
    const std::string test_server_name_shm = "-rgbd";

    rgbd::Client client;
};

TEST_F(Connection, ConsistentConnection)
{
    rgbd::Image image;
    double last_time_stamp = 0;
    EXPECT_FALSE(ros::isShuttingDown());
    ros::Time start = ros::Time::now();
    ros::Time shm_kill_time = start + ros::Duration(5);
    bool shm_killed = false;
    ros::Time end = start + ros::Duration(20);
    ros::Rate r(1);
    int i=1;
    while(ros::Time::now() < end)
    {
        if (!shm_killed && ros::Time::now() >= shm_kill_time)
        {
            ipc::shared_memory_object::remove(test_server_name_shm.c_str());
            shm_killed = true;
            ros::Duration(1).sleep(); // Wait for RGBD client to take over
            /* Reset rate, otherwise first sleep is zero, because of the longer sleep above.
             * Which eliminates any time between the two consecutive nextImage calls.
             * The second call will fail then, because the elapsed time has been to short to receive any new image.
             */
            r.reset();
        }
        EXPECT_TRUE(client.nextImage(image)) << "i=" << i;
        ++i;
        EXPECT_GT(image.getTimestamp(), last_time_stamp);
        last_time_stamp = image.getTimestamp();
        r.sleep();
    }
    EXPECT_FALSE(ros::isShuttingDown());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "connection_client");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

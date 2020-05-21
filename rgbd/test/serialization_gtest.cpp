#include <gtest/gtest.h>

TEST(serialization, True)
{
    EXPECT_TRUE(true);
}

TEST(deserialization, True)
{
    EXPECT_TRUE(true);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

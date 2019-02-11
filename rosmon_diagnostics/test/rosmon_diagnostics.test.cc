#include "../src/resource_checker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../src/octets_parse.h"

#define DEFAULT_MEM_LIMIT 150000
#define DEFAULT_CPU_LIMIT 0.2

TEST(Rosmon_Diagnostics, WeCanParseMemorySize)
{
    OctetsParser parser;
    uint64_t memory;
    bool ok = parser.parseMemory("1  o", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1);

    ok = parser.parseMemory("100", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 100);

    ok = parser.parseMemory("1 Mo", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1e6);

    ok = parser.parseMemory("1Mo", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1e6);

    ok = parser.parseMemory("1.6  Go", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1.6e9);

    ok = parser.parseMemory("1 go", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1.0e9);
}

TEST(Rosmon_Diagnostics, WeExtractResourcesLimitsFromParameters)
{
    rosmon_diagnostics::ResourceChecker resourceChecker;

    ASSERT_EQ(resourceChecker.getMaxAllowedMemory("unknown_node"), DEFAULT_MEM_LIMIT);
    ASSERT_FLOAT_EQ(resourceChecker.getMaxUserAllowedCPU("unknown_node"),
                    DEFAULT_CPU_LIMIT);

    ASSERT_EQ(resourceChecker.getMaxAllowedMemory("foo_node"), 250000);
    ASSERT_EQ(resourceChecker.getMaxAllowedMemory("bar_node"), 4500000);
    ASSERT_FLOAT_EQ(resourceChecker.getMaxUserAllowedCPU("foo_node"), 0.6);
    ASSERT_FLOAT_EQ(resourceChecker.getMaxUserAllowedCPU("bar_node"), DEFAULT_CPU_LIMIT);
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rosmon_diag_tester");
    return RUN_ALL_TESTS();
    return 0;
}

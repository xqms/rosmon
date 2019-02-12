#include "../src/octets_parse.h"
#include "../src/resource_checker.h"
#include "../src/rosmon_to_diagnostic.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

#define DEFAULT_MEM_LIMIT 150000
#define DEFAULT_CPU_LIMIT 0.2

using namespace rosmon_diagnostics;
using namespace rosmon_msgs;

TEST(Rosmon_Diagnostics, WeCanParseMemorySize)
{
    OctetsParser parser;
    uint64_t memory;
    bool ok = parser.parseMemory("1  B", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1);

    ok = parser.parseMemory("100", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 100);

    ok = parser.parseMemory("1 MB", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1e6);

    ok = parser.parseMemory("1MB", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1e6);

    ok = parser.parseMemory("1.6  GB", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1.6e9);

    ok = parser.parseMemory("1 gB", memory);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1.0e9);
}

TEST(Rosmon_Diagnostics, WeFormatCorrectlyMemorySizes)
{
    ASSERT_STREQ(RosmonToDiagnostic::memoryToString(123).c_str(), "123 Bytes");
    ASSERT_STREQ(RosmonToDiagnostic::memoryToString(12345).c_str(), "12.35 KB");
    ASSERT_STREQ(RosmonToDiagnostic::memoryToString(1500000).c_str(), "1.50 MB");
    ASSERT_STREQ(RosmonToDiagnostic::memoryToString(1288000000).c_str(), "1.29 GB");
}

NodeState createNodeState(const std::string& name, uint8_t state = NodeState::RUNNING,
                          uint64_t memory = 1, float cpu = 0.05, int restartCount = 0)
{
    NodeState nodeState;
    nodeState.name = name;
    nodeState.state = state;
    nodeState.restart_count = restartCount;
    nodeState.user_load = cpu;
    nodeState.memory = memory;
    return nodeState;
}

TEST(Rosmon_Diagnostics, WeConvertToDiagnosticsInAComplexScenario)
{
    // Given a complex scenario in wich :
    // * good_node has no problem
    // * foo_node use too much cpu (> 0.6)
    // * bar_node use too much memory (4.5 MB)
    // * restarted_node has a non empty restart counter
    // * crashed_node is crashed !
    State state;
    state.header.stamp = ros::Time(10.0);
    state.nodes.push_back(createNodeState("good_node"));
    state.nodes.push_back(createNodeState("foo_node", NodeState::RUNNING, 1, 0.8, 0));
    state.nodes.push_back(createNodeState("bar_node", NodeState::RUNNING, 5e6, 0.05, 0));
    state.nodes.push_back(
        createNodeState("restarted_node", NodeState::RUNNING, 1, 0.05, 10));
    state.nodes.push_back(
        createNodeState("crashed_node", NodeState::CRASHED, 1, 0.05, 0));

    RosmonToDiagnostic rosmonToDiagnostic;
    rosmonToDiagnostic.onNewStateMessage(state);
    auto diag = rosmonToDiagnostic.getCurrentDiagnosticArray();

    EXPECT_DOUBLE_EQ(diag.header.stamp.toSec(), 10.);

    for(auto diagStatus : diag.status)
    {
        std::cerr << "Node : " << diagStatus.name << " message : " << diagStatus.message << std::endl;
        if(diagStatus.name == "procs/good_node")
        {
            EXPECT_EQ(diagStatus.level, diagnostic_msgs::DiagnosticStatus::OK);
        }
        else if(diagStatus.name == "procs/foo_node" ||
                diagStatus.name == "procs/bar_node" ||
                diagStatus.name == "procs/restarted_node")
        {
            EXPECT_EQ(diagStatus.level, diagnostic_msgs::DiagnosticStatus::WARN);
        }
        else if(diagStatus.name == "procs/crashed_node")
        {
            EXPECT_EQ(diagStatus.level, diagnostic_msgs::DiagnosticStatus::ERROR);
        }
        else
        {
            // unknwon node name !
            EXPECT_TRUE(false);
        }
    }
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

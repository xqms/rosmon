#include <gtest/gtest.h>
#include "../src/diagnostics/rosmon_to_diagnostic.h"
#include "../src/launch/bytes_parser.h"

#define DEFAULT_MEM_LIMIT 150000
#define DEFAULT_CPU_LIMIT 0.2

using namespace rosmon::diagnostics;
using namespace rosmon::launch;

TEST(Rosmon_Diagnostics, WeCanParseMemorySize)
{
    ByteParser parser;
    auto res = parser.parseMemory("1  B");
    uint64_t memory = std::get<0>(res);
    bool ok = std::get<1>(res);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1);

    res = parser.parseMemory("100");
    memory = std::get<0>(res);
    ok = std::get<1>(res);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 100);

    res = parser.parseMemory("1 MB");
    memory = std::get<0>(res);
    ok = std::get<1>(res);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1e6);

    res = parser.parseMemory("1MB");
    memory = std::get<0>(res);
    ok = std::get<1>(res);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1e6);

    res = parser.parseMemory("1.6  GB");
    memory = std::get<0>(res);
    ok = std::get<1>(res);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1.6e9);

    res = parser.parseMemory("1 gB");
    memory = std::get<0>(res);
    ok = std::get<1>(res);
    EXPECT_TRUE(ok);
    EXPECT_EQ(memory, 1.0e9);
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

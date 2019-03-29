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
    ASSERT_STREQ(ByteStringGenerator::memoryToString(123).c_str(), "123 Bytes");
    ASSERT_STREQ(ByteStringGenerator::memoryToString(12345).c_str(), "12.35 KB");
    ASSERT_STREQ(ByteStringGenerator::memoryToString(1500000).c_str(), "1.50 MB");
    ASSERT_STREQ(ByteStringGenerator::memoryToString(1288000000).c_str(), "1.29 GB");
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

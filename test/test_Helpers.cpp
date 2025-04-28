#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../tasks/Helpers.hpp"
using namespace motors_roboteq_canopen::helpers;

using testing::ElementsAre;

struct HelpersTest : public ::testing::Test {};

TEST_F(HelpersTest, it_computes_digital_vector_difference)
{
    auto now = base::Time::now();
    std::vector<raw_io::Digital> s1{
        {now,  true},
        {now,  true},
        {now, false},
        {now,  true}
    };

    std::vector<std::uint8_t> diff = difference(s1, s1);
    EXPECT_EQ(0, diff.size());

    std::vector<raw_io::Digital> s2{
        {now, false},
        {now,  true},
        {now, false},
        {now, false}
    };

    diff = difference(s1, s2);
    EXPECT_THAT(diff, ElementsAre(0, 3));
}

TEST_F(HelpersTest, it_computes_managed_outputs_mask)
{
    // 1000 0100 0000 0101
    EXPECT_EQ(0x8405, managedDigitalOutputMask({1, 3, 11, 16}));
}

TEST_F(HelpersTest, it_computes_raw_command_from_config)
{
    std::uint16_t cmd = commandToRaw({0, 1, 0, 1}, {1, 3, 11, 16});
    // 1000 0000 0000 0100
    EXPECT_EQ(0x8004, cmd);
}
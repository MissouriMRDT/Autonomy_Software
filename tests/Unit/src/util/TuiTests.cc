/******************************************************************************
 * @brief Unit tests for TUI components (TuiLogBuffer, SystemMetricsCollector).
 *
 * @file TuiTests.cc
 * @author Missouri MRDT
 * @date 2026-09-21
 *
 * @copyright Copyright Mars Rover Design Team 2026 - All Rights Reserved
 ******************************************************************************/

#include "../../../../src/util/tui/TuiLogSink.h"
#include "../../../../src/util/tui/SystemMetrics.h"
#include "../../../TestingBase.hh"

#include <gtest/gtest.h>

/******************************************************************************
 * @brief Unit Test Class for TUI
 ******************************************************************************/
class TuiTestsTest : public TestingBase<TuiTestsTest>
{
    protected:
        void SetUp() override {}
        void TearDown() override {}
};

/******************************************************************************
 * @brief Test that TuiLogBuffer correctly pushes, limits capacity, and preserves FIFO.
 ******************************************************************************/
TEST_F(TuiTestsTest, LogBufferCapacityAndOrder)
{
    tui::TuiLogBuffer qBuffer(5);

    EXPECT_EQ(qBuffer.GetTotalCount(), 0);
    EXPECT_TRUE(qBuffer.GetSnapshot().empty());

    for (int i = 0; i < 10; ++i)
    {
        qBuffer.Push("12:00:0" + std::to_string(i),
                     quill::LogLevel::Info,
                     "Main",
                     "Message " + std::to_string(i),
                     "[INFO] Message " + std::to_string(i));
    }

    EXPECT_EQ(qBuffer.GetTotalCount(), 10);
    auto vSnapshot = qBuffer.GetSnapshot();
    EXPECT_EQ(vSnapshot.size(), 5);

    // Oldest should be Message 5, newest Message 9
    EXPECT_EQ(vSnapshot.front().szMessage, "Message 5");
    EXPECT_EQ(vSnapshot.back().szMessage, "Message 9");

    qBuffer.Clear();
    EXPECT_EQ(qBuffer.GetTotalCount(), 0);
    EXPECT_TRUE(qBuffer.GetSnapshot().empty());
}

/******************************************************************************
 * @brief Test that TuiLogBuffer filters by minimum log level correctly.
 ******************************************************************************/
TEST_F(TuiTestsTest, LogBufferLevelFiltering)
{
    tui::TuiLogBuffer qBuffer(20);

    qBuffer.Push("T1", quill::LogLevel::Debug, "Test", "Debug msg", "D: Debug msg");
    qBuffer.Push("T2", quill::LogLevel::Info, "Test", "Info msg", "I: Info msg");
    qBuffer.Push("T3", quill::LogLevel::Warning, "Test", "Warning msg", "W: Warning msg");
    qBuffer.Push("T4", quill::LogLevel::Error, "Test", "Error msg", "E: Error msg");

    auto vAll = qBuffer.GetSnapshot(quill::LogLevel::TraceL3);
    EXPECT_EQ(vAll.size(), 4);

    auto vInfoAndAbove = qBuffer.GetSnapshot(quill::LogLevel::Info);
    EXPECT_EQ(vInfoAndAbove.size(), 3);
    EXPECT_EQ(vInfoAndAbove[0].szMessage, "Info msg");
    EXPECT_EQ(vInfoAndAbove[1].szMessage, "Warning msg");
    EXPECT_EQ(vInfoAndAbove[2].szMessage, "Error msg");

    auto vErrorOnly = qBuffer.GetSnapshot(quill::LogLevel::Error);
    EXPECT_EQ(vErrorOnly.size(), 1);
    EXPECT_EQ(vErrorOnly[0].szMessage, "Error msg");
}

/******************************************************************************
 * @brief Test that SystemMetricsCollector queries hardware stats safely without throwing.
 ******************************************************************************/
TEST_F(TuiTestsTest, SystemMetricsQuery)
{
    tui::SystemMetricsCollector qCollector;
    tui::HardwareStats stStats = qCollector.Query();

    // Usage should be bounded between 0 and 100%
    EXPECT_GE(stStats.fCpuTotalUsage, 0.0f);
    EXPECT_LE(stStats.fCpuTotalUsage, 100.0f);

    // RAM total should be positive on any Linux system
    EXPECT_GE(stStats.fRamTotalGB, 0.0f);
    EXPECT_GE(stStats.fRamUsedGB, 0.0f);
}

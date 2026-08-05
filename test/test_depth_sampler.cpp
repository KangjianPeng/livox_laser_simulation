#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "livox_laser_simulation/livox_depth_sampler.hh"

namespace livox_laser_simulation
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

TEST(LivoxDepthSampler, SamplesGpuPixelsAndPreservesMetadata)
{
  LivoxDepthSampler sampler;
  ASSERT_TRUE(sampler.LoadScanPattern(TEST_SCAN_PATTERN, 4, 100.0));
  sampler.SetParameters(1, 0.1, 40.0, false);
  ASSERT_TRUE(sampler.SetDepthImageAngles(-kPi, kPi, -0.1, 0.1));
  ASSERT_TRUE(sampler.ScanPatternFitsDepthImage());

  std::vector<double> ranges(5 * 3, 10.0);
  const auto points =
    sampler.SampleDepthImage(ranges.data(), 5, 3, 0, 4);
  ASSERT_EQ(points.size(), 4u);

  EXPECT_NEAR(points[0].point.X(), 10.0, 1e-9);
  EXPECT_NEAR(points[0].point.Y(), 0.0, 1e-9);
  EXPECT_NEAR(points[1].point.X(), 0.0, 1e-9);
  EXPECT_NEAR(points[1].point.Y(), 10.0, 1e-9);
  EXPECT_NEAR(points[2].point.X(), -10.0, 1e-9);
  EXPECT_NEAR(points[3].point.Y(), -10.0, 1e-9);

  EXPECT_EQ(points[0].line, 0u);
  EXPECT_EQ(points[1].line, 1u);
  EXPECT_EQ(points[2].line, 2u);
  EXPECT_EQ(points[3].line, 3u);
  EXPECT_EQ(points[0].offset_time, 0u);
  EXPECT_EQ(points[1].offset_time, 10000000u);
  EXPECT_EQ(points[3].offset_time, 30000000u);
}

TEST(LivoxDepthSampler, RejectsUncoveredPatternAndInvalidRanges)
{
  LivoxDepthSampler sampler;
  ASSERT_TRUE(sampler.LoadScanPattern(TEST_SCAN_PATTERN, 1, 100.0));
  sampler.SetParameters(1, 0.1, 40.0, false);
  ASSERT_TRUE(sampler.SetDepthImageAngles(-1.0, 1.0, -0.1, 0.1));
  EXPECT_FALSE(sampler.ScanPatternFitsDepthImage());

  ASSERT_TRUE(sampler.SetDepthImageAngles(-kPi, kPi, -0.1, 0.1));
  std::vector<double> ranges(5 * 3, 10.0);
  ranges[2 + 5] = std::numeric_limits<double>::infinity();
  const auto points =
    sampler.SampleDepthImage(ranges.data(), 5, 3, 0, 4);
  EXPECT_EQ(points.size(), 3u);
}
}  // namespace
}  // namespace livox_laser_simulation

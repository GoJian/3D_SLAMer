#include <gtest/gtest.h>

#include <vector>

#include "x650_camera/v4l2_timing.hpp"

using namespace x650_camera;

TEST(V4l2Timing, MonotonicMapsToRealtimeWithCurrentOffset)
{
  // Frame captured 12 ms before the clock pair was read.
  const Nanoseconds mono_now = 5'000'000'000, real_now = 1'800'000'000'000'000'000;
  EXPECT_EQ(monotonicToRealtime(mono_now - 12'000'000, mono_now, real_now), real_now - 12'000'000);
}

TEST(V4l2Timing, TimevalToNs)
{
  timeval tv{3, 250'000};
  EXPECT_EQ(toNs(tv), 3'250'000'000);
}

TEST(V4l2Timing, StampReferenceFromFlags)
{
  EXPECT_EQ(stampReference(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_SOE),
    "exposure_start");
  EXPECT_EQ(stampReference(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_EOF),
    "readout_end");
  EXPECT_TRUE(isMonotonicStamp(V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC));
  EXPECT_FALSE(isMonotonicStamp(V4L2_BUF_FLAG_TIMESTAMP_COPY));
}

TEST(V4l2Timing, YuyvToMonoTakesLumaAndHonorsStride)
{
  // 2x2 image, stride 6 bytes (4 data + 2 padding): Y0 U Y1 V | pad
  const std::vector<std::uint8_t> src = {10, 128, 20, 128, 0, 0, 30, 128, 40, 128, 0, 0};
  std::vector<std::uint8_t> dst(4);
  yuyvToMono8(src.data(), 2, 2, 6, dst.data());
  EXPECT_EQ(dst, (std::vector<std::uint8_t>{10, 20, 30, 40}));
}

TEST(V4l2Timing, Fourcc)
{
  EXPECT_EQ(fourcc("YUYV"), static_cast<std::uint32_t>(V4L2_PIX_FMT_YUYV));
  EXPECT_EQ(fourcc("GREY"), static_cast<std::uint32_t>(V4L2_PIX_FMT_GREY));
  EXPECT_EQ(fourcc("BAD"), 0u);
}

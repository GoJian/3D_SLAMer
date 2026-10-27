// Pure helpers for V4L2 (USB webcam) capture: clock mapping, stamp semantics, pixel conversion.
// Kept free of ioctl code so they can be unit-tested without a device.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <linux/videodev2.h>

namespace x650_camera
{

using Nanoseconds = std::int64_t;

inline Nanoseconds toNs(const timeval & tv)
{
  return static_cast<Nanoseconds>(tv.tv_sec) * 1'000'000'000LL +
         static_cast<Nanoseconds>(tv.tv_usec) * 1'000LL;
}

// V4L2 buffer stamps are CLOCK_MONOTONIC. Map to CLOCK_REALTIME (ROS system time) using a pair of
// clock readings taken back to back; the offset changes only with NTP/chrony steps and slews.
inline Nanoseconds monotonicToRealtime(Nanoseconds mono, Nanoseconds mono_now, Nanoseconds real_now)
{
  return mono + (real_now - mono_now);
}

// What instant the driver's buffer timestamp refers to (V4L2_BUF_FLAG_TSTAMP_SRC_*), in the
// FrameMetadata.stamp_reference vocabulary.
inline std::string stampReference(std::uint32_t flags)
{
  switch (flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) {
    case V4L2_BUF_FLAG_TSTAMP_SRC_SOE: return "exposure_start";
    case V4L2_BUF_FLAG_TSTAMP_SRC_EOF: return "readout_end";
    default: return "unknown";
  }
}

inline bool isMonotonicStamp(std::uint32_t flags)
{
  return (flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
}

// YUYV (YUV 4:2:2, Y0 U Y1 V) -> 8-bit luma. `src_stride` is bytes per source row.
inline void yuyvToMono8(
  const std::uint8_t * src, std::size_t width, std::size_t height, std::size_t src_stride,
  std::uint8_t * dst)
{
  for (std::size_t r = 0; r < height; ++r) {
    const std::uint8_t * s = src + r * src_stride;
    std::uint8_t * d = dst + r * width;
    for (std::size_t c = 0; c < width; ++c) {
      d[c] = s[2 * c];
    }
  }
}

inline std::uint32_t fourcc(const std::string & s)
{
  if (s.size() != 4) {
    return 0;
  }
  return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

}  // namespace x650_camera

// USB webcam (V4L2) capture for bench work before the ZED X is available.
//
//   out: <ns>/image_raw        sensor_msgs/Image (mono8 from YUYV/GREY, or yuv422_yuy2 passthrough)
//        <ns>/frame_metadata   x650_msgs/FrameMetadata (V4L2 sequence, sensor stamp, exposure)
//        /diagnostics          diagnostic_msgs/DiagnosticArray
//
// header.stamp is the driver's buffer timestamp (CLOCK_MONOTONIC, taken by the kernel at frame
// capture) mapped to CLOCK_REALTIME, never the time this node received the frame. For UVC
// webcams the driver reports start of frame ("exposure_start"); the exact meaning is in
// FrameMetadata.stamp_reference. No hardware trigger: two webcams are NOT a synchronized stereo
// pair, and the audit tool will show their pairing offsets.
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "x650_camera/v4l2_timing.hpp"
#include "x650_common/time_mapping.hpp"
#include "x650_msgs/msg/frame_metadata.hpp"

namespace x650_camera
{
namespace
{
int xioctl(int fd, unsigned long req, void * arg)
{
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

Nanoseconds clockNs(clockid_t id)
{
  timespec ts{};
  clock_gettime(id, &ts);
  return static_cast<Nanoseconds>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

builtin_interfaces::msg::Time toMsg(Nanoseconds ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1'000'000'000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
  return t;
}
}  // namespace

class UsbCameraNode : public rclcpp::Node
{
public:
  explicit UsbCameraNode(const rclcpp::NodeOptions & options)
  : Node("usb_camera", options)
  {
    device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    camera_id_ = declare_parameter<std::string>("camera_id", "cam0");
    frame_id_ = declare_parameter<std::string>("frame_id", camera_id_ + "_optical");
    width_ = static_cast<std::uint32_t>(declare_parameter<int>("width", 640));
    height_ = static_cast<std::uint32_t>(declare_parameter<int>("height", 480));
    fps_ = declare_parameter<int>("fps", 30);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "YUYV");
    output_ = declare_parameter<std::string>("output_encoding", "mono8");
    exposure_us_ = declare_parameter<double>("exposure_us", -1.0);  // <0: leave camera on auto
    num_buffers_ = static_cast<std::uint32_t>(declare_parameter<int>("num_buffers", 4));
    if (pixel_format_ != "YUYV" && pixel_format_ != "GREY") {
      throw std::invalid_argument("pixel_format must be YUYV or GREY (MJPEG is not decoded here)");
    }
    if (output_ != "mono8" && !(output_ == "yuv422_yuy2" && pixel_format_ == "YUYV")) {
      throw std::invalid_argument("output_encoding must be mono8, or yuv422_yuy2 with YUYV input");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(5);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    meta_pub_ = create_publisher<x650_msgs::msg::FrameMetadata>("~/frame_metadata", qos);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    open();
    running_ = true;
    thread_ = std::thread([this] {loop();});
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publishDiagnostics();});
  }

  ~UsbCameraNode() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    close();
  }

private:
  struct Buffer
  {
    void * start{nullptr};
    std::size_t length{0};
  };

  void fail(const std::string & what)
  {
    throw std::runtime_error(device_ + ": " + what + ": " + std::strerror(errno));
  }

  void open()
  {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      fail("open");
    }
    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
      fail("VIDIOC_QUERYCAP");
    }
    if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) || !(cap.device_caps & V4L2_CAP_STREAMING)) {
      throw std::runtime_error(device_ + " is not a streaming capture device");
    }
    card_ = reinterpret_cast<const char *>(cap.card);

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = fourcc(pixel_format_);
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      fail("VIDIOC_S_FMT");
    }
    if (fmt.fmt.pix.pixelformat != fourcc(pixel_format_)) {
      throw std::runtime_error(device_ + " does not support " + pixel_format_);
    }
    if (fmt.fmt.pix.width != width_ || fmt.fmt.pix.height != height_) {
      RCLCPP_WARN(get_logger(), "requested %ux%u, driver chose %ux%u", width_, height_,
        fmt.fmt.pix.width, fmt.fmt.pix.height);
      width_ = fmt.fmt.pix.width;
      height_ = fmt.fmt.pix.height;
    }
    stride_ = fmt.fmt.pix.bytesperline;

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<std::uint32_t>(fps_);
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
      RCLCPP_WARN(get_logger(), "could not set %d fps: %s", fps_, std::strerror(errno));
    }

    if (exposure_us_ > 0.0) {
      // Fixed exposure for VIO: auto exposure changes brightness frame to frame.
      v4l2_control c{V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL};
      if (xioctl(fd_, VIDIOC_S_CTRL, &c) < 0) {
        RCLCPP_WARN(get_logger(), "could not disable auto exposure: %s", std::strerror(errno));
      }
      v4l2_control e{V4L2_CID_EXPOSURE_ABSOLUTE, static_cast<int>(std::lround(exposure_us_ / 100.0))};
      if (xioctl(fd_, VIDIOC_S_CTRL, &e) < 0) {
        RCLCPP_WARN(get_logger(), "could not set exposure: %s", std::strerror(errno));
      }
    }

    v4l2_requestbuffers req{};
    req.count = num_buffers_;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
      fail("VIDIOC_REQBUFS");
    }
    buffers_.resize(req.count);
    for (std::uint32_t i = 0; i < req.count; ++i) {
      v4l2_buffer b{};
      b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = i;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &b) < 0) {
        fail("VIDIOC_QUERYBUF");
      }
      buffers_[i].length = b.length;
      buffers_[i].start = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        fail("mmap");
      }
      if (xioctl(fd_, VIDIOC_QBUF, &b) < 0) {
        fail("VIDIOC_QBUF");
      }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      fail("VIDIOC_STREAMON");
    }
    RCLCPP_INFO(get_logger(), "%s (%s): %ux%u %s @ %d fps -> %s", device_.c_str(), card_.c_str(),
      width_, height_, pixel_format_.c_str(), fps_, output_.c_str());
  }

  void close()
  {
    if (fd_ < 0) {
      return;
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    for (auto & b : buffers_) {
      if (b.start && b.start != MAP_FAILED) {
        munmap(b.start, b.length);
      }
    }
    ::close(fd_);
    fd_ = -1;
  }

  float exposureUs()
  {
    v4l2_control c{V4L2_CID_EXPOSURE_ABSOLUTE, 0};
    return xioctl(fd_, VIDIOC_G_CTRL, &c) == 0 ? 100.0f * static_cast<float>(c.value) : NAN;
  }

  void loop()
  {
    while (running_ && rclcpp::ok()) {
      pollfd p{fd_, POLLIN, 0};
      const int r = poll(&p, 1, 500);
      if (r == 0) {
        ++timeouts_;
        continue;
      }
      if (r < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "poll: %s", std::strerror(errno));
        break;
      }
      v4l2_buffer b{};
      b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
        if (errno != EAGAIN) {
          RCLCPP_ERROR(get_logger(), "VIDIOC_DQBUF: %s", std::strerror(errno));
        }
        continue;
      }
      handle(b);
      if (xioctl(fd_, VIDIOC_QBUF, &b) < 0) {
        RCLCPP_ERROR(get_logger(), "VIDIOC_QBUF: %s", std::strerror(errno));
      }
    }
  }

  void handle(const v4l2_buffer & b)
  {
    const Nanoseconds mono_now = clockNs(CLOCK_MONOTONIC);
    const Nanoseconds real_now = clockNs(CLOCK_REALTIME);
    Nanoseconds sensor = toNs(b.timestamp);
    std::string reference = stampReference(b.flags);
    std::string domain = "host_monotonic";
    if (!isMonotonicStamp(b.flags) || sensor <= 0) {
      // Driver gave no usable capture stamp: fall back to arrival, and say so in the metadata.
      sensor = mono_now;
      reference = "arrival";
      ++arrival_stamped_;
    }
    const Nanoseconds stamp = monotonicToRealtime(sensor, mono_now, real_now);
    last_age_ns_ = mono_now - sensor;
    if (b.flags & V4L2_BUF_FLAG_ERROR) {
      ++corrupt_;
    }
    {
      std::lock_guard<std::mutex> lk(seq_mutex_);
      seq_.update(b.sequence);
    }

    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->header.stamp = toMsg(stamp);
    img->header.frame_id = frame_id_;
    img->width = width_;
    img->height = height_;
    const auto * src = static_cast<const std::uint8_t *>(buffers_[b.index].start);
    if (output_ == "mono8") {
      img->encoding = "mono8";
      img->step = width_;
      img->data.resize(static_cast<std::size_t>(width_) * height_);
      if (pixel_format_ == "YUYV") {
        yuyvToMono8(src, width_, height_, stride_, img->data.data());
      } else {
        for (std::uint32_t r = 0; r < height_; ++r) {
          std::memcpy(img->data.data() + r * width_, src + r * stride_, width_);
        }
      }
    } else {
      img->encoding = "yuv422_yuy2";
      img->step = stride_;
      img->data.assign(src, src + static_cast<std::size_t>(stride_) * height_);
    }

    x650_msgs::msg::FrameMetadata md;
    md.header = img->header;
    md.camera_id = camera_id_;
    md.sequence = b.sequence;
    md.sensor_stamp_ns = sensor;
    md.clock_domain = domain;
    md.stamp_reference = reference;
    md.exposure_us = exposureUs();
    md.analog_gain_db = NAN;
    md.digital_gain = NAN;
    image_pub_->publish(std::move(img));
    meta_pub_->publish(md);
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "usb_camera " + camera_id_;
    s.hardware_id = device_ + " " + card_;
    auto kv = [&s](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        s.values.push_back(e);
      };
    std::uint64_t received, dropped;
    {
      std::lock_guard<std::mutex> lk(seq_mutex_);
      received = seq_.received();
      dropped = seq_.dropped();
    }
    const bool frames = received > last_received_;
    s.level = frames ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    s.message = frames ? "streaming" : "no frames";
    kv("frames", std::to_string(received));
    kv("dropped", std::to_string(dropped));
    kv("corrupt", std::to_string(corrupt_.load()));
    kv("arrival_stamped", std::to_string(arrival_stamped_.load()));
    kv("last_capture_to_dequeue_s", std::to_string(1e-9 * static_cast<double>(last_age_ns_.load())));
    last_received_ = received;
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    a.status.push_back(s);
    diag_pub_->publish(a);
  }

  std::string device_, camera_id_, frame_id_, pixel_format_, output_, card_;
  std::uint32_t width_{640}, height_{480}, stride_{0}, num_buffers_{4};
  int fps_{30};
  double exposure_us_{-1.0};
  int fd_{-1};
  std::vector<Buffer> buffers_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex seq_mutex_;
  x650_common::SequenceTracker seq_;   // capture thread writes, diagnostics timer reads
  std::atomic<std::uint64_t> corrupt_{0}, arrival_stamped_{0}, timeouts_{0};
  std::atomic<Nanoseconds> last_age_ns_{0};
  std::uint64_t last_received_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<x650_msgs::msg::FrameMetadata>::SharedPtr meta_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace x650_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(x650_camera::UsbCameraNode)

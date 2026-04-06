#include "Camera.hpp"

#include "MissionConstants.hpp"
#include "WhiteCircle.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <iomanip>
#include <ios>
#include <mutex>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <thread>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{
struct VideoStreamState
{
    bool initialized = false;
    cv::VideoCapture capture;
    int activeDeviceIndex = -1;
};

VideoStreamState gVideoStream;

std::string DevicePathForIndex(int deviceIndex)
{
    return std::string("/dev/video") + std::to_string(deviceIndex);
}

bool IsCaptureDeviceIndex(int deviceIndex)
{
    const std::string devicePath = DevicePathForIndex(deviceIndex);
    const int fd = ::open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        return false;
    }

    v4l2_capability caps{};
    const int ioctlResult = ::ioctl(fd, VIDIOC_QUERYCAP, &caps);
    ::close(fd);
    if (ioctlResult < 0) {
        return false;
    }

    uint32_t deviceCaps = caps.capabilities;
    if (caps.capabilities & V4L2_CAP_DEVICE_CAPS) {
        deviceCaps = caps.device_caps;
    }

    const bool hasCapture = (deviceCaps & V4L2_CAP_VIDEO_CAPTURE) ||
                            (deviceCaps & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
    const bool hasStreaming = (deviceCaps & V4L2_CAP_STREAMING);
    return hasCapture && hasStreaming;
}

struct DebugFrameItem
{
    double frameId = -1.0;
    cv::Mat frame;
};

class DebugFrameLogger
{
public:
    DebugFrameLogger()
        : worker(&DebugFrameLogger::Run, this)
    {
    }

    ~DebugFrameLogger()
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
        }
        condition.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
    }

    void Enqueue(double frameId, const cv::Mat& frame)
    {
        if (!MissionConstants::kSensorCameraSaveDebugFrames) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex);
        if (queue.size() >= kMaxQueueDepth) {
            queue.pop_front();
        }
        queue.push_back(DebugFrameItem{frameId, frame.clone()});
        condition.notify_one();
    }

private:
    static constexpr size_t kMaxQueueDepth = 8;

    void Run()
    {
        const std::filesystem::path debugDir = std::filesystem::absolute(
            MissionConstants::kSensorCameraDebugFrameDirectory).lexically_normal();
        std::cerr << "Camera debug frame output directory: " << debugDir.string() << std::endl;

        std::error_code createDirectoryError;
        std::filesystem::create_directories(
            debugDir,
            createDirectoryError);
        if (createDirectoryError) {
            std::cerr << "Camera debug frame directory creation failed: "
                      << debugDir.string()
                      << " (" << createDirectoryError.message() << ")" << std::endl;
        }

        for (;;) {
            DebugFrameItem item;
            {
                std::unique_lock<std::mutex> lock(mutex);
                condition.wait(lock, [&] { return stop || !queue.empty(); });
                if (stop && queue.empty()) {
                    return;
                }

                item = std::move(queue.front());
                queue.pop_front();
            }

            std::ostringstream filename;
            filename << debugDir.string() << "/"
                     << "frame_" << std::setw(6) << std::setfill('0') << static_cast<int>(item.frameId)
                     << ".jpg";

            if (!cv::imwrite(filename.str(), item.frame)) {
                std::cerr << "Camera debug frame write failed: " << filename.str() << std::endl;
            } else {
                std::cerr << "Camera debug frame saved: " << filename.str() << std::endl;
            }
        }
    }

    std::mutex mutex;
    std::condition_variable condition;
    std::deque<DebugFrameItem> queue;
    bool stop = false;
    std::thread worker;
};

DebugFrameLogger& GetDebugFrameLogger()
{
    static DebugFrameLogger logger;
    return logger;
}

Eigen::Vector3d PixelToUnitVector(double px, double py)
{
    const cv::Matx33d cameraMatrix(
        MissionConstants::kSensorCameraFocalLengthXPx, 0.0, MissionConstants::kSensorCameraPrincipalPointXPx,
        0.0, MissionConstants::kSensorCameraFocalLengthYPx, MissionConstants::kSensorCameraPrincipalPointYPx,
        0.0, 0.0, 1.0);
    const cv::Vec<double, 5> distortion(
        MissionConstants::kSensorCameraDistortionK1,
        MissionConstants::kSensorCameraDistortionK2,
        MissionConstants::kSensorCameraDistortionP1,
        MissionConstants::kSensorCameraDistortionP2,
        MissionConstants::kSensorCameraDistortionK3);

    std::vector<cv::Point2f> distortedPoints;
    distortedPoints.emplace_back(static_cast<float>(px), static_cast<float>(py));
    std::vector<cv::Point2f> undistortedPoints;
    cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix, distortion);

    if (undistortedPoints.empty()) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d ray(
        static_cast<double>(undistortedPoints[0].x),
        static_cast<double>(undistortedPoints[0].y),
        1.0);
    return ray.normalized();
}
}

Camera::Camera()
{
}

bool Camera::InitializeVideoStream()
{
    if (gVideoStream.initialized && gVideoStream.capture.isOpened()) {
        return true;
    }

    if (gVideoStream.capture.isOpened()) {
        gVideoStream.capture.release();
    }
    gVideoStream.activeDeviceIndex = -1;

    const int preferredDeviceIndex = MissionConstants::kSensorCameraDeviceIndex;
    std::vector<int> deviceCandidates;
    deviceCandidates.push_back(preferredDeviceIndex);
    for (int idx = 0; idx <= 5; ++idx) {
        if (idx != preferredDeviceIndex) {
            deviceCandidates.push_back(idx);
        }
    }

    const std::array<std::pair<int, int>, 3> resolutionCandidates = {
        std::make_pair(MissionConstants::kSensorCameraImageWidthPx, MissionConstants::kSensorCameraImageHeightPx),
        std::make_pair(1280, 720),
        std::make_pair(640, 480),
    };

    auto tryOpenCapture = [&](int deviceIndex, int width, int height) -> bool {
        gVideoStream.capture.release();

        if (MissionConstants::kSensorCameraUseGStreamer) {
            std::cerr << "Camera trying GStreamer pipeline at " << width << "x" << height << " on device index " << deviceIndex << std::endl;
            std::ostringstream pipeline;
            // Use camera-name=... if you need to select a specific port by index (e.g. camera-name=0 or camera-name=1) 
            pipeline
                << "libcamerasrc camera-name=" << deviceIndex << " ! video/x-raw,width=" << width
                << ",height=" << height
                << ",format=RGBx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";

            if (gVideoStream.capture.open(pipeline.str(), cv::CAP_GSTREAMER)) {
                std::cerr << "Camera opened with GStreamer pipeline at " << width << "x" << height << std::endl;
                gVideoStream.capture.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
                
                cv::Mat warmupFrame;
                bool warmupSucceeded = false;
                for (int attempt = 0; attempt < 20; ++attempt) {
                    if (gVideoStream.capture.read(warmupFrame) && !warmupFrame.empty()) {
                        warmupSucceeded = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (warmupSucceeded) {
                    std::cerr << "Camera warmup succeeded via GStreamer on device index " << deviceIndex
                              << " at " << width << "x" << height << std::endl;
                    gVideoStream.activeDeviceIndex = deviceIndex;
                    gVideoStream.initialized = true;
                    return true;
                }
                
                std::cerr << "Camera warmup failed via GStreamer at " << width << "x" << height << std::endl;
                gVideoStream.capture.release();
            } else {
                std::cerr << "Camera GStreamer pipeline failed to open at " << width << "x" << height << std::endl;
            }
            
            // Second try without camera-name just in case index lookup fails
            std::cerr << "Camera trying generic GStreamer pipeline at " << width << "x" << height << std::endl;
            std::ostringstream genericPipeline;
            genericPipeline
                << "libcamerasrc ! video/x-raw,width=" << width
                << ",height=" << height
                << ",format=RGBx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1";
                
            if (gVideoStream.capture.open(genericPipeline.str(), cv::CAP_GSTREAMER)) {
                std::cerr << "Camera opened with generic GStreamer pipeline at " << width << "x" << height << std::endl;
                gVideoStream.capture.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
                
                cv::Mat warmupFrame;
                bool warmupSucceeded = false;
                for (int attempt = 0; attempt < 20; ++attempt) {
                    if (gVideoStream.capture.read(warmupFrame) && !warmupFrame.empty()) {
                        warmupSucceeded = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if (warmupSucceeded) {
                    std::cerr << "Camera warmup succeeded via generic GStreamer at " << width << "x" << height << std::endl;
                    gVideoStream.activeDeviceIndex = deviceIndex;
                    gVideoStream.initialized = true;
                    return true;
                }
                gVideoStream.capture.release();
            }
        }

        std::cerr << "Camera trying device index " << deviceIndex
                  << " at " << width << "x" << height << std::endl;
        if (!gVideoStream.capture.open(deviceIndex, cv::CAP_V4L2)) {
            if (MissionConstants::kSensorCameraUseCapAnyFallback) {
                if (!gVideoStream.capture.open(deviceIndex, cv::CAP_ANY)) {
                    std::cerr << "Camera open failed for device index " << deviceIndex << std::endl;
                    return false;
                }
            } else {
                std::cerr << "Camera open failed for device index " << deviceIndex << " using V4L2" << std::endl;
                return false;
            }
        }

        std::cerr << "Camera opened on device index " << deviceIndex
                  << " with requested resolution " << width << "x" << height << std::endl;

        gVideoStream.capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
        gVideoStream.capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        gVideoStream.capture.set(cv::CAP_PROP_BUFFERSIZE, 1.0);

        // Grab one frame at startup so the next request returns a recent image.
        cv::Mat warmupFrame;
        bool warmupSucceeded = false;
        for (int attempt = 0; attempt < 20; ++attempt) {
            if (gVideoStream.capture.read(warmupFrame) && !warmupFrame.empty()) {
                warmupSucceeded = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (warmupSucceeded) {
            std::cerr << "Camera warmup succeeded on device index " << deviceIndex
                      << " at " << width << "x" << height << std::endl;
            gVideoStream.activeDeviceIndex = deviceIndex;
            gVideoStream.initialized = true;
            return true;
        }

        std::cerr << "Camera warmup frame read failed on device index " << deviceIndex
                  << " at " << width << "x" << height << std::endl;
        return false;
    };

    for (const int deviceIndex : deviceCandidates) {
        if (!MissionConstants::kSensorCameraUseGStreamer && !IsCaptureDeviceIndex(deviceIndex)) {
            std::cerr << "Camera skipping device index " << deviceIndex
                      << " because it is not a V4L2 capture device" << std::endl;
            continue;
        }

        for (const auto& resolution : resolutionCandidates) {
            if (tryOpenCapture(deviceIndex, resolution.first, resolution.second)) {
                return true;
            }
        }
    }

    gVideoStream.capture.release();
    gVideoStream.initialized = false;
    return false;
}

void Camera::TryProcessPendingLocalCapture()
{
    if (!capturePending) {
        return;
    }

    if (CaptureLocalFrameAndProcess(pendingCaptureFrameId)) {
        capturePending = false;
    }
}

bool Camera::CaptureLocalFrameAndProcess(double frameId)
{
    if (!InitializeVideoStream()) {
        return false;
    }

    cv::Mat img;
    if (!gVideoStream.capture.read(img)) {
        std::cerr << "Camera frame read failed on device index " << gVideoStream.activeDeviceIndex << std::endl;
        gVideoStream.initialized = false;
        return false;
    }

    // Drop one buffered frame when available to bias toward the latest image.
    cv::Mat latestImg;
    if (gVideoStream.capture.grab()) {
        gVideoStream.capture.retrieve(latestImg);
        if (!latestImg.empty()) {
            img = latestImg;
        }
    }

    if (img.empty()) {
        std::cerr << "Camera captured empty frame on device index " << gVideoStream.activeDeviceIndex << std::endl;
        return false;
    }

    std::vector<WhiteCircle::MarkerDetection> detections = WhiteCircle::DetectWhiteMarkerCentroids(
        img,
        MissionConstants::kSensorCameraMarkerMinAreaPx,
        MissionConstants::kSensorCameraMaxDetections);

    GetDebugFrameLogger().Enqueue(frameId, img);

    std::vector<std::pair<double, double>> pixelList;
    pixelList.reserve(detections.size());
    for (const WhiteCircle::MarkerDetection& detection : detections) {
        pixelList.emplace_back(detection.centroidPx.x, detection.centroidPx.y);
    }

    UpdateFromPixelList(frameId, pixelList);
    return true;
}

void Camera::RequestCapture()
{
    ++nextCaptureFrameId;
    pendingCaptureFrameId = nextCaptureFrameId;
    capturePending = true;
}

void Camera::UpdateFromPixelList(double frameId, const std::vector<std::pair<double, double>>& pixelList)
{
    latestUnitVectorList.clear();
    const size_t n = std::min<size_t>(
        static_cast<size_t>(MissionConstants::kSensorCameraMaxDetections),
        pixelList.size());
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d unitVector = PixelToUnitVector(pixelList[i].first, pixelList[i].second);
        if (unitVector.norm() > 0.5) {
            latestUnitVectorList.push_back(unitVector);
        }
    }
    latestFrameId = frameId;
}

std::vector<Eigen::Vector3d> Camera::GetUnitVectorList()
{
    TryProcessPendingLocalCapture();
    return latestUnitVectorList;
}

double Camera::GetFrameId()
{
    TryProcessPendingLocalCapture();
    return latestFrameId;
}

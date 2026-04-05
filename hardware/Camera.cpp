#include "Camera.hpp"

#include "MissionConstants.hpp"
#include "WhiteCircle.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct VideoStreamState
{
    bool initialized = false;
    cv::VideoCapture capture;
};

VideoStreamState gVideoStream;

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

    if (!gVideoStream.capture.open(0, cv::CAP_V4L2)) {
        if (!gVideoStream.capture.open(0, cv::CAP_ANY)) {
            gVideoStream.initialized = false;
            return false;
        }
    }

    gVideoStream.capture.set(cv::CAP_PROP_FRAME_WIDTH, MissionConstants::kSensorCameraImageWidthPx);
    gVideoStream.capture.set(cv::CAP_PROP_FRAME_HEIGHT, MissionConstants::kSensorCameraImageHeightPx);
    gVideoStream.capture.set(cv::CAP_PROP_BUFFERSIZE, 1.0);

    // Grab one frame at startup so the next request returns a recent image.
    cv::Mat warmupFrame;
    gVideoStream.capture.read(warmupFrame);

    gVideoStream.initialized = true;
    return true;
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
        return false;
    }

    std::vector<WhiteCircle::MarkerDetection> detections = WhiteCircle::DetectWhiteMarkerCentroids(
        img,
        MissionConstants::kSensorCameraMarkerMinAreaPx,
        MissionConstants::kSensorCameraMaxDetections);

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

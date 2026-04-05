#include "Camera.hpp"

#include "MissionConstants.hpp"
#include "WhiteCircle.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
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
    std::ostringstream command;
    command << MissionConstants::kSensorCameraCaptureCommand
            << " -n"
            << " --timeout " << MissionConstants::kSensorCameraCaptureTimeoutMs
            << " --width " << MissionConstants::kSensorCameraImageWidthPx
            << " --height " << MissionConstants::kSensorCameraImageHeightPx
            << " -o " << MissionConstants::kSensorCameraCaptureOutputPath
            << " > /dev/null 2>&1";

    const int ret = std::system(command.str().c_str());
    if (ret != 0) {
        return false;
    }

    cv::Mat img = cv::imread(MissionConstants::kSensorCameraCaptureOutputPath, cv::IMREAD_COLOR);
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

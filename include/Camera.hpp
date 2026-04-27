#pragma once

#include <Eigen/Dense>

#include <string>
#include <tuple>
#include <utility>
#include <vector>

class Camera
{
public:
    Camera();
    bool getUseCamera() { return useCamera; }
    void setUseCamera(const bool &state) { useCamera = state; }
    void RequestCapture();
    std::vector<Eigen::Vector3d> GetUnitVectorList();
    double GetFrameId();
    void AnnotateDebugFrameMatches(
        double frameId,
        const std::vector<std::pair<int, int>> &measurementToMarkerMatches);
    void AnnotateDebugFrameExpectedVsTrue(
        double frameId,
        const std::vector<std::pair<int, Eigen::Vector3d>> &expectedMarkerBodyDirections);

private:
    std::vector<Eigen::Vector3d> latestUnitVectorList;
    bool useCamera = true;
    double latestFrameId = -1.0;
    bool capturePending = false;
    double nextCaptureFrameId = 0.0;
    double pendingCaptureFrameId = -1.0;

    void TryProcessPendingLocalCapture();
    bool InitializeVideoStream();
    bool CaptureLocalFrameAndProcess(double frameId);
    void UpdateFromPixelList(double frameId, const std::vector<std::pair<double, double>> &pixelList);
};
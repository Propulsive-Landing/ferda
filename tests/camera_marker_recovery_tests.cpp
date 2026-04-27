#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>
#include <vector>

#include "CameraRecovery.hpp"

namespace
{
std::vector<Eigen::Vector3d> BuildMeasuredDirections(const Eigen::Vector3d &vehiclePosition)
{
    std::vector<Eigen::Vector3d> measuredDirections;
    measuredDirections.reserve(MissionConstants::kMarkerData.cols());

    for (int markerIdx = 0; markerIdx < MissionConstants::kMarkerData.cols(); ++markerIdx)
    {
        const Eigen::Vector3d markerWorld = MarkerRecovery::MarkerWorldPosition(markerIdx);
        measuredDirections.push_back((markerWorld - vehiclePosition).normalized());
    }

    return measuredDirections;
}
} // namespace

int main()
{
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d truePosition(1.25, -0.35, 2.5);

    {
        std::vector<Eigen::Vector3d> measuredDirections = BuildMeasuredDirections(truePosition);
        std::mt19937 rng(7);
        std::shuffle(measuredDirections.begin(), measuredDirections.end(), rng);

        const MarkerRecovery::RecoveryResult result = MarkerRecovery::RecoverPositionFromDetectedMarkers(measuredDirections, R);
        assert(result.valid);
        assert((result.recoveredPosition - truePosition).norm() < 1e-4);
        assert(result.averageAngularErrorRad < MissionConstants::kNavCameraRecoveryMaxAverageAngularErrorRad);
        assert(result.measurementToMarkerMatches.size() == static_cast<size_t>(MissionConstants::kMarkerData.cols()));
    }

    {
        std::vector<Eigen::Vector3d> measuredDirections = BuildMeasuredDirections(truePosition);
        measuredDirections.resize(3);

        const MarkerRecovery::RecoveryResult result = MarkerRecovery::RecoverPositionFromDetectedMarkers(measuredDirections, R);
        assert(!result.valid);
    }

    {
        std::vector<Eigen::Vector3d> measuredDirections = BuildMeasuredDirections(truePosition);
        measuredDirections[0] = (-measuredDirections[0]).normalized();

        const MarkerRecovery::RecoveryResult result = MarkerRecovery::RecoverPositionFromDetectedMarkers(measuredDirections, R);
        assert(!result.valid || result.averageAngularErrorRad > MissionConstants::kNavCameraRecoveryMaxAverageAngularErrorRad);
    }

    std::cout << "Camera marker recovery tests passed.\n";
    return 0;
}
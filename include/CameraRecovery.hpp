#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

#include "MissionConstants.hpp"

namespace MarkerRecovery
{
struct RecoveryResult
{
    bool valid = false;
    Eigen::Vector3d recoveredPosition = Eigen::Vector3d::Zero();
    double averageAngularErrorRad = std::numeric_limits<double>::infinity();
    std::vector<std::pair<int, int>> measurementToMarkerMatches;
};

inline double AngularErrorRad(const Eigen::Vector3d &measured, const Eigen::Vector3d &predicted)
{
    const double cosTheta = std::clamp(measured.dot(predicted), -1.0, 1.0);
    return std::acos(cosTheta);
}

inline Eigen::Vector3d MarkerWorldPosition(int markerIdx)
{
    return MissionConstants::kMarkerData.col(markerIdx) + MissionConstants::kStructuresGroundOffset;
}

inline int CountBits(unsigned int value)
{
    int count = 0;
    while (value != 0U)
    {
        count += static_cast<int>(value & 1U);
        value >>= 1U;
    }
    return count;
}

inline bool SolvePositionFromCorrespondences(
    const std::vector<Eigen::Vector3d> &measuredBody,
    const std::vector<int> &measurementIndices,
    const std::vector<int> &markerIndices,
    const Eigen::Matrix3d &R,
    Eigen::Vector3d &recoveredPosition)
{
    if (measurementIndices.empty() || measurementIndices.size() != markerIndices.size())
    {
        return false;
    }

    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    const Eigen::Vector3d cameraOffsetWorld = R * MissionConstants::kSensorCameraPosition;
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    for (size_t i = 0; i < measurementIndices.size(); ++i)
    {
        const Eigen::Vector3d measured = measuredBody[measurementIndices[i]];
        if (measured.norm() < 1e-9)
        {
            return false;
        }

        const Eigen::Vector3d rayWorld = (R * measured).normalized();
        const Eigen::Vector3d anchorWorld = MarkerWorldPosition(markerIndices[i]) - cameraOffsetWorld;
        const Eigen::Matrix3d projection = identity - rayWorld * rayWorld.transpose();
        A += projection;
        b += projection * anchorWorld;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.singularValues()(2) < 1e-8)
    {
        return false;
    }

    recoveredPosition = svd.solve(b);
    return recoveredPosition.allFinite();
}

inline RecoveryResult RecoverPositionFromDetectedMarkers(
    const std::vector<Eigen::Vector3d> &measuredBody,
    const Eigen::Matrix3d &R)
{
    RecoveryResult best;
    const int detectionCount = static_cast<int>(measuredBody.size());
    const int markerCount = MissionConstants::kMarkerData.cols();

    if (detectionCount < MissionConstants::kNavCameraRecoveryMinDetectedMarkers ||
        detectionCount < markerCount)
    {
        return best;
    }

    const unsigned int combinationLimit = 1u << detectionCount;
    for (unsigned int mask = 0; mask < combinationLimit; ++mask)
    {
        if (CountBits(mask) != markerCount)
        {
            continue;
        }

        std::vector<int> measurementIndices;
        measurementIndices.reserve(markerCount);
        for (int measurementIdx = 0; measurementIdx < detectionCount; ++measurementIdx)
        {
            if (mask & (1u << measurementIdx))
            {
                measurementIndices.push_back(measurementIdx);
            }
        }

        std::vector<int> markerIndices(markerCount);
        std::iota(markerIndices.begin(), markerIndices.end(), 0);

        do
        {
            Eigen::Vector3d recoveredPosition = Eigen::Vector3d::Zero();
            if (!SolvePositionFromCorrespondences(measuredBody, measurementIndices, markerIndices, R, recoveredPosition))
            {
                continue;
            }

            double scoreSum = 0.0;
            bool validCandidate = true;
            std::vector<std::pair<int, int>> measurementToMarkerMatches;
            measurementToMarkerMatches.reserve(markerCount);

            for (int i = 0; i < markerCount; ++i)
            {
                const int measurementIdx = measurementIndices[i];
                const int markerIdx = markerIndices[i];
                const Eigen::Vector3d delta =
                    R.transpose() * (MarkerWorldPosition(markerIdx) - recoveredPosition) -
                    MissionConstants::kSensorCameraPosition;
                const double deltaNorm = delta.norm();
                if (deltaNorm < 1e-9)
                {
                    validCandidate = false;
                    break;
                }

                const Eigen::Vector3d predictedBody = delta / deltaNorm;
                scoreSum += AngularErrorRad(measuredBody[measurementIdx], predictedBody);
                measurementToMarkerMatches.emplace_back(measurementIdx, markerIdx);
            }

            if (!validCandidate)
            {
                continue;
            }

            const double averageAngularErrorRad = scoreSum / static_cast<double>(markerCount);
            if (!best.valid || averageAngularErrorRad < best.averageAngularErrorRad)
            {
                best.valid = true;
                best.recoveredPosition = recoveredPosition;
                best.averageAngularErrorRad = averageAngularErrorRad;
                best.measurementToMarkerMatches = std::move(measurementToMarkerMatches);
            }
        } while (std::next_permutation(markerIndices.begin(), markerIndices.end()));
    }

    return best;
}
} // namespace MarkerRecovery
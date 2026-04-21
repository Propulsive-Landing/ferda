#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <deque>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <numeric>

#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include <iostream>

namespace
{
    struct PredictedMarker
    {
        int markerIdx = -1;
        double d = 0.0;
        Eigen::Vector3d rho = Eigen::Vector3d::Zero();
        Eigen::Vector3d u = Eigen::Vector3d::Zero();
    };

    double AngularErrorRad(const Eigen::Vector3d &measured, const Eigen::Vector3d &predicted)
    {
        const double cosTheta = std::clamp(measured.dot(predicted), -1.0, 1.0);
        return std::acos(cosTheta);
    }

    bool SolveHungarianRectangular(
        const std::vector<std::vector<double>> &costs,
        std::vector<int> &assignment,
        double &totalCost)
    {
        if (costs.empty() || costs.front().empty())
        {
            return false;
        }

        const int rows = static_cast<int>(costs.size());
        const int cols = static_cast<int>(costs.front().size());
        if (rows > cols)
        {
            return false;
        }

        for (int r = 0; r < rows; ++r)
        {
            if (static_cast<int>(costs[r].size()) != cols)
            {
                return false;
            }
        }

        std::vector<double> u(rows + 1, 0.0);
        std::vector<double> v(cols + 1, 0.0);
        std::vector<int> p(cols + 1, 0);
        std::vector<int> way(cols + 1, 0);

        for (int i = 1; i <= rows; ++i)
        {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(cols + 1, std::numeric_limits<double>::infinity());
            std::vector<bool> used(cols + 1, false);

            do
            {
                used[j0] = true;
                const int i0 = p[j0];
                double delta = std::numeric_limits<double>::infinity();
                int j1 = 0;

                for (int j = 1; j <= cols; ++j)
                {
                    if (used[j])
                    {
                        continue;
                    }

                    const double cur = costs[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j])
                    {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta)
                    {
                        delta = minv[j];
                        j1 = j;
                    }
                }

                if (!std::isfinite(delta))
                {
                    return false;
                }

                for (int j = 0; j <= cols; ++j)
                {
                    if (used[j])
                    {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    }
                    else
                    {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);

            do
            {
                const int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        assignment.assign(rows, -1);
        for (int j = 1; j <= cols; ++j)
        {
            if (p[j] != 0)
            {
                assignment[p[j] - 1] = j - 1;
            }
        }

        totalCost = 0.0;
        for (int i = 0; i < rows; ++i)
        {
            if (assignment[i] < 0 || assignment[i] >= cols)
            {
                return false;
            }
            totalCost += costs[i][assignment[i]];
        }

        return std::isfinite(totalCost);
    }

    bool AssignByHungarian(
        const std::vector<std::vector<double>> &costs,
        std::vector<int> &assignment,
        double &totalCost)
    {
        if (costs.empty() || costs.front().empty())
        {
            return false;
        }

        const int numMeasurements = static_cast<int>(costs.size());
        const int numPredictions = static_cast<int>(costs.front().size());

        std::vector<std::vector<double>> augmentedCosts(
            numMeasurements,
            std::vector<double>(numPredictions + numMeasurements,
                                MissionConstants::kNavCameraAssociationUnassignedPenaltyRad));

        for (int i = 0; i < numMeasurements; ++i)
        {
            for (int j = 0; j < numPredictions; ++j)
            {
                augmentedCosts[i][j] = costs[i][j];
            }
        }

        std::vector<int> rawAssignment;
        if (!SolveHungarianRectangular(augmentedCosts, rawAssignment, totalCost))
        {
            return false;
        }

        assignment.assign(numMeasurements, -1);
        for (int i = 0; i < numMeasurements; ++i)
        {
            if (rawAssignment[i] >= 0 && rawAssignment[i] < numPredictions)
            {
                assignment[i] = rawAssignment[i];
            }
        }

        return true;
    }

    bool AssignCameraMarkers(
        const std::vector<Eigen::Vector3d> &measuredBody,
        const std::vector<PredictedMarker> &predictions,
        std::vector<int> &assignment,
        double &totalCost)
    {
        const int numMeasurements = static_cast<int>(measuredBody.size());
        const int numPredictions = static_cast<int>(predictions.size());
        if (numMeasurements == 0 || numPredictions == 0)
        {
            return false;
        }

        std::vector<std::vector<double>> costs(
            numMeasurements,
            std::vector<double>(numPredictions, 0.0));

        for (int measurementIdx = 0; measurementIdx < numMeasurements; ++measurementIdx)
        {
            for (int predictionIdx = 0; predictionIdx < numPredictions; ++predictionIdx)
            {
                costs[measurementIdx][predictionIdx] =
                    AngularErrorRad(measuredBody[measurementIdx], predictions[predictionIdx].rho);
            }
        }

        return AssignByHungarian(costs, assignment, totalCost);
    }
} // namespace

Navigation::Navigation(IMU &inputImu, Magnetometer &inputMagnetometer, GPS &inputGps, Lidar &inputLidar, Camera &inputCamera, TVC &inputTvc) : imu(inputImu), magnetometer(inputMagnetometer), gps(inputGps), lidar(inputLidar), camera(inputCamera), tvc(inputTvc)
{
    stateMat = Eigen::Matrix<double, 16, 1>::Zero();
    // Initializes Quaternion to [1,0,0,0] equivalent to 0 roll, 0 pitch, 0 yaw
    stateMat(6) = 1;

    Eigen::VectorXd d(15);
    d << MissionConstants::kNavInitialPositionVariance, MissionConstants::kNavInitialPositionVariance, MissionConstants::kNavInitialPositionVariance,   // Position variances
        MissionConstants::kNavInitialVelocityVariance, MissionConstants::kNavInitialVelocityVariance, MissionConstants::kNavInitialVelocityVariance,    // Velocity variances
        MissionConstants::kNavInitialAttitudeVariance, MissionConstants::kNavInitialAttitudeVariance, MissionConstants::kNavInitialAttitudeVariance,    // Attitude variances
        MissionConstants::kNavInitialAccelBiasVariance, MissionConstants::kNavInitialAccelBiasVariance, MissionConstants::kNavInitialAccelBiasVariance, // Accelerometer bias variances
        MissionConstants::kNavInitialGyroBiasVariance, MissionConstants::kNavInitialGyroBiasVariance, MissionConstants::kNavInitialGyroBiasVariance;    // Gyroscope bias variances

    P = d.asDiagonal();
    estimatedMassKg = MissionConstants::kStructuresWetMassKg;
    estimatedMassFraction = 1.0;
    UpdateMassPropertyEstimates();

    // Logging for SIL testing
    // dataFile.open("data.csv", std::ios::app);
}

void Navigation::reset()
{
    estimatedMassKg = MissionConstants::kStructuresWetMassKg;
    estimatedMassFraction = 1.0;
    gps_update_counter = 0;
    lidar_update_counter = 0;
    magnetometer_update_counter = 0;
    camera_capture_elapsed_s = 0.0;
    last_camera_frame_id = -1.0;
    UpdateMassPropertyEstimates();
}

Eigen::Matrix<double, 16, 1> Navigation::GetNavigation()
{
    return stateMat;
}

Eigen::Vector3d Navigation::GetAngularVelocity()
{
    return w;
}

// Function to write a double to a CSV file
void writeDoubleToCSV(double myDouble1, double myDouble2, double myDouble3, double myDouble4, double myDouble5, double myDouble6, int precision = 6)
{
    // Open the CSV file in append mode
    std::ofstream file;
    file.open("data.csv", std::ios::app); // Append mode

    // Check if the file is open
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file for writing.\n";
        return; // Exit the function if the file couldn't be opened
    }

    // Set the desired precision and write the double to the file
    file << std::fixed << std::setprecision(precision) << myDouble1 << "," << myDouble2 << "," << myDouble3 << "," << myDouble4 << "," << myDouble5 << "," << myDouble6 << "\n";

    // Close the file
    file.close();
}

void Navigation::UpdateNavigation()
{
    // Extract states from stateMat //
    x_e = stateMat.segment(0, 3);
    v_e = stateMat.segment(3, 3);
    q = Eigen::Quaterniond(
        stateMat(6), // w
        stateMat(7), // x
        stateMat(8), // y
        stateMat(9)  // z
    );
    q.normalize();
    a_b = stateMat.segment(10, 3);
    w_b = stateMat.segment(13, 3);

    Eigen::Matrix3d R = q.toRotationMatrix();
    const double g = MissionConstants::kGravity; // m/s^2
    Eigen::Vector3d g_vec(0, 0, -g);

    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu
    linearAcceleration = imu.GetBodyAcceleration();
    angularRate = imu.GetBodyAngularRate();

    Eigen::Vector3d a_m(std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration));
    Eigen::Vector3d w_m(std::get<0>(angularRate), std::get<1>(angularRate), std::get<2>(angularRate));

    // std::cout << a_b.transpose() << ", " << w_b.transpose() << "\n";

    // Logging for SIL testing
    // dataFile << std::fixed << std::setprecision(6)
    //          << x_e(0) << "," << x_e(1) << "," << x_e(2) << ","
    //          << v_e(0) << "," << v_e(1) << "," << v_e(2) << "\n";

    // Nominal State Calculation //
    x_e += v_e * loopTime + 0.5 * (R * (a_m - a_b) + g_vec) * loopTime * loopTime;
    v_e += (R * (a_m - a_b) + g_vec) * loopTime;

    // Small-angle quaternion
    Eigen::Vector3d theta = (w_m - w_b) * loopTime;
    double angle = theta.norm();

    Eigen::Quaterniond dq;
    if (angle < 1e-12)
    {
        dq = Eigen::Quaterniond::Identity();
    }
    else
    {
        dq = Eigen::AngleAxisd(angle, theta / angle);
    }

    q = (q * dq).normalized();

    // Covariance Calculation //

    Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(15, 15);

    Fx.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * loopTime;
    Fx.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    Fx.block<3, 3>(3, 6) = -R * skew(a_m - a_b) * loopTime;
    Fx.block<3, 3>(3, 9) = -R * loopTime;
    Fx.block<3, 3>(6, 6) = dq.toRotationMatrix().transpose();
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * loopTime;
    Fx.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
    Fx.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd Fi = Eigen::MatrixXd::Zero(15, 12);
    Fi.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
    Fi.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
    Fi.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    Fi.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    const double sigma_a_n = MissionConstants::kNavAccelWhiteNoiseSigma;
    const double sigma_w_n = MissionConstants::kNavGyroWhiteNoiseSigma;
    const double sigma_a_w = MissionConstants::kNavAccelBiasRandomWalkSigma;
    const double sigma_w_w = MissionConstants::kNavGyroBiasRandomWalkSigma;

    Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(12, 12);
    Qi.block<3, 3>(0, 0) = sigma_a_n * sigma_a_n * loopTime * loopTime * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = sigma_w_n * sigma_w_n * loopTime * loopTime * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = sigma_a_w * sigma_a_w * loopTime * loopTime * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = sigma_w_w * sigma_w_w * loopTime * loopTime * Eigen::Matrix3d::Identity();

    // Compute Covariance Matrix //
    P = Fx * P * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Update state estimates with available measurements

    // Update magnetometer on fixed cadence
    ++magnetometer_update_counter;
    if (magnetometer_update_counter >= kMagnetometerUpdateCadence && false)
    {
        magneticField = magnetometer.GetMagneticField();
        Eigen::Vector3d magneticFieldVector(std::get<0>(magneticField), std::get<1>(magneticField), std::get<2>(magneticField));
        magnetometerUpdate(magneticFieldVector, R);
        magnetometer_update_counter = 0; // Reset counter
    }
    ++gps_update_counter;
    if (gps_update_counter == kGPSUpdateCadence)
    {
        if (gps.GPSUsed())
        {

            gps.Update();

            if (gps.GPSAvailable())
            {
                // Make Sure GPS position is new and is toggled to be used
                if (gps.HasFreshPosition() && gps.GetUseGPSPosition())
                {
                    gpsPosition = gps.GetGPSPosition();
                    Eigen::Vector3d gpsPositionVector(std::get<0>(gpsPosition), std::get<1>(gpsPosition), std::get<2>(gpsPosition));
                    gpsPositionUpdate(gpsPositionVector);
                }

                // Make Sure GPS velocity is new and is toggled to be used
                if (gps.HasFreshVelocity() && gps.GetUseGPSVelocity())
                {
                    gpsVelocity = gps.GetGPSVelocity();
                    Eigen::Vector2d gpsVelocityVector(std::get<0>(gpsVelocity), std::get<1>(gpsVelocity));
                    gpsVelocityUpdate(gpsVelocityVector);
                }
            }
        }
        gps_update_counter = 0;
    }

    // TODO: GO OVER TOGGLE LOGIC FOR CAMREA BELOW TO MAKE SURE IT IS RIGHT
    camera_capture_elapsed_s += loopTime;
    if (camera_capture_elapsed_s >= kCameraCapturePeriodS)
    {
        // Only continue the process if Camera is toggled to be on
        if (camera.getUseCamera())
        {
            camera.RequestCapture();
        }
        camera_capture_elapsed_s -= kCameraCapturePeriodS;
    }

    // Only continue the process if Camera is toggled to be on
    if (camera.getUseCamera())
    {
        const std::vector<Eigen::Vector3d> cameraDirections = camera.GetUnitVectorList();
        const double camera_frame_id = camera.GetFrameId();
        if (camera_frame_id >= 0.0 && camera_frame_id != last_camera_frame_id)
        {
            cameraUpdate(cameraDirections, R, camera_frame_id);
            last_camera_frame_id = camera_frame_id;
        }
    }

    // Update lidar on fixed cadence near ground.
    ++lidar_update_counter;
    if (lidar_update_counter >= kLidarUpdateCadence && false)
    {
        if (x_e(2) < 4.0)
        {
            double lidarDistance = std::get<0>(lidar.GetLidarDistance());
            lidarUpdate(lidarDistance, R);
        }
        lidar_update_counter = 0; // Reset counter
    }

    Eigen::Vector3d angularRateVector = Eigen::Vector3d(std::get<0>(angularRate), std::get<1>(angularRate), std::get<2>(angularRate));
    w = angularRateVector;

    // Apply pad updates when on the pad (idle mode)
    if (onPad)
    {
        padUpdateVelocity();
        padUpdateAngularVelocity(w);
    }

    // Repack states into stateMat
    stateMat.segment(0, 3) = x_e;
    stateMat.segment(3, 3) = v_e;
    stateMat(6) = q.w();
    stateMat(7) = q.x();
    stateMat(8) = q.y();
    stateMat(9) = q.z();
    stateMat.segment(10, 3) = a_b;
    stateMat.segment(13, 3) = w_b;
}

void Navigation::magnetometerUpdate(const Eigen::Vector3d &magneticField, const Eigen::Matrix3d &R)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 6) = skew(R.transpose() * MissionConstants::kEarthMagField);
    Eigen::Vector3d y_pred = R.transpose() * MissionConstants::kEarthMagField;

    const double mag_noise = MissionConstants::kSensorMagnetometerNoise;
    Eigen::MatrixXd V = MissionConstants::kNavMagnetometerNoiseFactor *
                        mag_noise * mag_noise * Eigen::Matrix3d::Identity();

    kalmanUpdate(H, V, magneticField, y_pred);
}

void Navigation::gpsPositionUpdate(const Eigen::Vector3d &gpsPosition)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    Eigen::VectorXd y = Eigen::VectorXd::Zero(3);
    y << gpsPosition(0), gpsPosition(1), gpsPosition(2);

    Eigen::VectorXd y_pred = Eigen::VectorXd::Zero(3);
    y_pred << x_e(0), x_e(1), x_e(2);

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(3, 3);
    V.block<3, 3>(0, 0) = MissionConstants::kNavGPSPositionNoiseFactor *
                          MissionConstants::kSensorGPSPositionNoiseM * MissionConstants::kSensorGPSPositionNoiseM *
                          Eigen::Matrix3d::Identity();

    kalmanUpdate(H, V, y, y_pred);
}

void Navigation::gpsVelocityUpdate(const Eigen::Vector2d &gpsVelocity)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 15);
    H.block<2, 2>(0, 3) = Eigen::Matrix2d::Identity();

    Eigen::VectorXd y = Eigen::VectorXd::Zero(2);
    y << gpsVelocity(0), gpsVelocity(1);

    Eigen::VectorXd y_pred = Eigen::VectorXd::Zero(2);
    y_pred << v_e(0), v_e(1);

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(2, 2);
    V.block<2, 2>(0, 0) = MissionConstants::kNavGPSVelocityNoiseFactor *
                          MissionConstants::kSensorGPSVelocityNoiseMps * MissionConstants::kSensorGPSVelocityNoiseMps *
                          Eigen::Matrix2d::Identity();

    kalmanUpdate(H, V, y, y_pred);
}

void Navigation::lidarUpdate(double lidar, const Eigen::Matrix3d &R)
{
    const Eigen::Vector3d sensor_lidar_dir_orig(0.0, 0.0, -1.0);
    const Eigen::Vector3d sensor_r_lidar_orig(0.2, 0.0, 0.5);

    Eigen::VectorXd y = Eigen::VectorXd::Zero(1);
    y(0) = lidar;

    // Distance to z=0 plane along the current lidar pointing ray.
    const Eigen::Vector3d pointing_dir = R * sensor_lidar_dir_orig;
    const Eigen::Vector3d sensor_loc = x_e + R * sensor_r_lidar_orig;
    if (std::abs(pointing_dir(2)) < 1e-6)
    {
        return;
    }

    Eigen::VectorXd y_pred = Eigen::VectorXd::Zero(1);
    y_pred(0) = sensor_loc(2);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 15);
    H(0, 2) = -1.0 / pointing_dir(2);

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(1, 1);
    V(0, 0) = MissionConstants::kNavLidarNoiseFactor *
              MissionConstants::kSensorLidarNoiseM *
              MissionConstants::kSensorLidarNoiseM;

    kalmanUpdate(H, V, y, y_pred);
}

void Navigation::padUpdateVelocity()
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Eigen::Vector3d initialVelocity = Eigen::Vector3d(0, 0, 0);
    // Zero-velocity update: measurement is zero, prediction is current estimated velocity.
    const double pad_velocity_variance = MissionConstants::kNavInitialVelocityVariance;
    kalmanUpdate(H, pad_velocity_variance * Eigen::Matrix3d::Identity(), initialVelocity, v_e);
}

void Navigation::padUpdateAngularVelocity(const Eigen::Vector3d &angularVelocity)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
    // On-pad pseudo-measurement model: measured gyro rate ~= gyro bias (true body rate ~= 0).
    Eigen::Vector3d predictedAngularVelocity = w_b;
    const double pad_angular_velocity_variance = MissionConstants::kNavGyroWhiteNoiseSigma * MissionConstants::kNavGyroWhiteNoiseSigma;
    kalmanUpdate(H, pad_angular_velocity_variance * Eigen::Matrix3d::Identity(), angularVelocity, predictedAngularVelocity);
}

void Navigation::SetOnPad(bool isOnPad)
{
    onPad = isOnPad;
}

void Navigation::cameraUpdate(const std::vector<Eigen::Vector3d> &cameraDirections, const Eigen::Matrix3d &R, double frameId)
{
    const int N = MissionConstants::kMarkerData.cols();

    const Eigen::Vector3d eul = MissionConstants::kSensorCameraOrientationRad;
    const Eigen::Matrix3d DCM_bc =
        (Eigen::AngleAxisd(eul.x(), Eigen::Vector3d::UnitX()).toRotationMatrix() *
         Eigen::AngleAxisd(eul.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(eul.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix());

    std::vector<Eigen::Vector3d> measuredBody;
    measuredBody.reserve(cameraDirections.size());
    for (const Eigen::Vector3d &measuredCam : cameraDirections)
    {
        if (measuredCam.norm() > 0.5)
        {
            measuredBody.push_back((DCM_bc * measuredCam).normalized());
        }
    }

    std::vector<PredictedMarker> predictions;
    predictions.reserve(N);
    for (int i = 0; i < N; ++i)
    {
        const Eigen::Vector3d delta =
            R.transpose() * (MissionConstants::kMarkerData.col(i) + MissionConstants::kStructuresGroundOffset - x_e) -
            MissionConstants::kSensorCameraPosition;

        const double d = delta.norm();
        if (d < 1e-9)
        {
            continue;
        }

        PredictedMarker prediction;
        prediction.markerIdx = i;
        prediction.d = d;
        prediction.rho = delta / d;
        prediction.u = (R.transpose() * delta) / d;
        predictions.push_back(prediction);
    }

    std::vector<std::pair<int, Eigen::Vector3d>> expectedMarkerBodyDirections;
    expectedMarkerBodyDirections.reserve(predictions.size());
    for (const auto &prediction : predictions)
    {
        expectedMarkerBodyDirections.emplace_back(prediction.markerIdx, prediction.rho);
    }
    camera.AnnotateDebugFrameExpectedVsTrue(frameId, expectedMarkerBodyDirections);

    const int M = static_cast<int>(measuredBody.size());
    if (M == 0)
    {
        camera.AnnotateDebugFrameMatches(frameId, {});
        return;
    }

    if (!predictions.empty())
    {
        std::cerr << "[NAV] Expected marker unit vectors:";
        for (const auto &prediction : predictions)
        {
            std::cerr << " [marker " << prediction.markerIdx
                      << " -> [" << prediction.rho.transpose() << "]]";
        }
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "[NAV] Expected marker unit vectors: none" << std::endl;
    }

    std::vector<int> assignment;
    double totalAngularError = std::numeric_limits<double>::infinity();
    if (!AssignCameraMarkers(measuredBody, predictions, assignment, totalAngularError))
    {
        camera.AnnotateDebugFrameMatches(frameId, {});
        return;
    }

    std::vector<std::pair<int, int>> matchedPairs;
    matchedPairs.reserve(static_cast<size_t>(M));

    for (int k = 0; k < M; ++k)
    {
        const int predictionIdx = assignment[k];
        if (predictionIdx < 0 || predictionIdx >= static_cast<int>(predictions.size()))
        {
            continue;
        }
        matchedPairs.emplace_back(k, predictionIdx);
    }

    std::vector<std::pair<int, int>> measurementToMarkerMatches;
    measurementToMarkerMatches.reserve(matchedPairs.size());
    for (const auto &matchedPair : matchedPairs)
    {
        const int measurementIdx = matchedPair.first;
        const int predictionIdx = matchedPair.second;
        measurementToMarkerMatches.emplace_back(measurementIdx, predictions[predictionIdx].markerIdx);
    }
    camera.AnnotateDebugFrameMatches(frameId, measurementToMarkerMatches);

    if (!matchedPairs.empty())
    {
        std::cerr << "[NAV] Matched marker pairs:";
        for (const auto &matchedPair : matchedPairs)
        {
            const int measurementIdx = matchedPair.first;
            const int predictionIdx = matchedPair.second;
            const PredictedMarker &prediction = predictions[predictionIdx];
            const Eigen::Vector3d measuredDirection = measuredBody[measurementIdx];
            const double angularErrorRad = AngularErrorRad(measuredDirection, prediction.rho);
            std::cerr << " [measurement " << measurementIdx
                      << " -> marker " << prediction.markerIdx
                      << ", measured=[" << measuredDirection.transpose() << "]"
                      << ", expected=[" << prediction.rho.transpose() << "]"
                      << ", error_rad=" << angularErrorRad
                      << ", error_deg=" << angularErrorRad * MissionConstants::kRad2Deg
                      << "]";
        }
        std::cerr << std::endl;
    }

    const int K = static_cast<int>(matchedPairs.size());

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3 * K, 15);
    Eigen::VectorXd y = Eigen::VectorXd::Zero(3 * K);
    Eigen::VectorXd y_pred = Eigen::VectorXd::Zero(3 * K);

    for (int k = 0; k < K; ++k)
    {
        const int measurementIdx = matchedPairs[k].first;
        const int predictionIdx = matchedPairs[k].second;
        const PredictedMarker &prediction = predictions[predictionIdx];
        const int row = 3 * k;

        y.segment<3>(row) = measuredBody[measurementIdx];
        y_pred.segment<3>(row) = prediction.rho;
        H.block<3, 3>(row, 0) =
            -R.transpose() * (Eigen::Matrix3d::Identity() - prediction.u * prediction.u.transpose()) / prediction.d;
        H.block<3, 3>(row, 6) = skew(R.transpose() * prediction.u);
    }

    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(3 * K, 3 * K);
    const double cameraNoise = MissionConstants::kNavCameraNoiseFactor *
                               MissionConstants::kSensorCameraNoise *
                               MissionConstants::kSensorCameraNoise;
    for (int k = 0; k < K; ++k)
    {
        V.block<3, 3>(3 * k, 3 * k) = cameraNoise * Eigen::Matrix3d::Identity();
    }

    kalmanUpdate(H, V, y, y_pred);
}

void Navigation::kalmanUpdate(
    const Eigen::MatrixXd &H,
    const Eigen::MatrixXd &V,
    const Eigen::VectorXd &y,
    const Eigen::VectorXd &y_pred)
{
    Eigen::VectorXd r = y - y_pred;
    Eigen::MatrixXd S = H * P * H.transpose() + V;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    Eigen::VectorXd dx = K * r;
    P = (Eigen::MatrixXd::Identity(15, 15) - K * H) * P;

    // Inject error state
    x_e += dx.segment<3>(0);
    v_e += dx.segment<3>(3);

    Eigen::Vector3d dtheta = dx.segment<3>(6);
    Eigen::Quaterniond dq(1, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
    q = (q * dq).normalized();

    a_b += dx.segment<3>(9);
    w_b += dx.segment<3>(12);
}

std::tuple<double, double, double> Navigation::GetLinearAcceleration()
{
    return linearAcceleration;
}

std::tuple<double, double, double> Navigation::GetAngularAcceleration()
{
    return angularRate;
}

void Navigation::UpdateMassFractionEstimate(double throttleCommandN)
{
    const double wetMassKg = MissionConstants::kStructuresWetMassKg;
    const double dryMassKg = MissionConstants::kStructuresDryMassKg;
    const double propellantMassKg = wetMassKg - dryMassKg;

    if (propellantMassKg <= 0.0)
    {
        estimatedMassKg = dryMassKg;
        estimatedMassFraction = 0.0;
        return;
    }

    const double commandedThrustN = std::max(0.0, throttleCommandN);
    const double massFlowRateKgPerS = commandedThrustN * MissionConstants::kThrottleToMassFlowScale;

    estimatedMassKg += massFlowRateKgPerS * loopTime;
    if (estimatedMassKg < dryMassKg)
    {
        estimatedMassKg = dryMassKg;
    }
    else if (estimatedMassKg > wetMassKg)
    {
        estimatedMassKg = wetMassKg;
    }

    estimatedMassFraction = (estimatedMassKg - dryMassKg) / propellantMassKg;
    UpdateMassPropertyEstimates();
}

void Navigation::UpdateMassPropertyEstimates()
{
    const double alpha = std::clamp(estimatedMassFraction, 0.0, 1.0);
    estimatedCenterOfMassBodyM = MissionConstants::kStructuresDryCenterOfMassBodyM +
                                 alpha * (MissionConstants::kStructuresWetCenterOfMassBodyM - MissionConstants::kStructuresDryCenterOfMassBodyM);
    estimatedMomentOfInertiaBodyKgm2 = MissionConstants::kStructuresDryMomentOfInertiaBodyKgm2 +
                                       alpha * (MissionConstants::kStructuresWetMomentOfInertiaBodyKgm2 - MissionConstants::kStructuresDryMomentOfInertiaBodyKgm2);
}

double Navigation::GetEstimatedMassFraction()
{
    return estimatedMassFraction;
}

double Navigation::GetEstimatedMassKg()
{
    return estimatedMassKg;
}

Eigen::Vector3d Navigation::GetEstimatedCenterOfMassBodyM()
{
    return estimatedCenterOfMassBodyM;
}

Eigen::Vector3d Navigation::GetEstimatedMomentOfInertiaBodyKgm2()
{
    return estimatedMomentOfInertiaBodyKgm2;
}

std::tuple<double, double, double> Navigation::GetMagneticField()
{
    return magneticField;
}

Eigen::Matrix3d Navigation::skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return S;
}

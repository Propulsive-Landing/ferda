#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <vector>
#include <iostream>
#include <chrono>
#include <iomanip>

#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include <iostream>

Navigation::Navigation(IMU &inputImu, Magnetometer &inputMagnetometer, GPS &inputGps, Camera &inputCamera, TVC &inputTvc) : imu(inputImu), magnetometer(inputMagnetometer), gps(inputGps), camera(inputCamera), tvc(inputTvc)
{
    std::cout << std::setprecision(4) << std::fixed;
    stateMat = Eigen::Matrix<double, 16, 1>::Zero();
    // Set z position to rocket com
    stateMat(2) = 0.28;
    // Initializes Quaternion to [1,0,0,0] equivalent to 0 roll, 0 pitch, 0 yaw
    stateMat(6) = 1;
    
    Eigen::VectorXd d(15);
    d << 1e-5, 1e-5, 1e-5, // Position variances
         0, 0, 0, // Velocity variances
         0, 0, 0, // Attitude variances
         1e-2, 1e-2, 1e-2, // Accelerometer bias variances
         1e-5, 1e-5, 1e-5; // Gyroscope bias variances

    P = d.asDiagonal();
}

void Navigation::reset()
{
    stateMat = Eigen::Matrix<double, 16, 1>::Zero();
    stateMat(2) = 0.28;
    stateMat(6) = 1;
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
        stateMat(6),   // w
        stateMat(7),   // x
        stateMat(8),   // y
        stateMat(9)    // z
    );
    q.normalize();
    a_b = stateMat.segment(10, 3);
    w_b = stateMat.segment(13, 3);

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d g(0, 0, -9.81);

    // Create 2 tuples to hold the the linear acceleration and angular rate data from the imu
    linearAcceleration = imu.GetBodyAcceleration();
    angularRate = imu.GetBodyAngularRate();

    // std::cout << "Accel Z:" << std::to_string(std::get<2>(linearAcceleration)) << " gyroX: " << std::to_string(std::get<0>(angularRate)) << "\n";
    // Convert the linear acceleration tuple to a Vector so we can muliply the Eigen matrix R by another Eigen type which in this case is a vector
    Eigen::Vector3d a_m(std::get<0>(linearAcceleration), std::get<1>(linearAcceleration), std::get<2>(linearAcceleration));
    Eigen::Vector3d w_m(std::get<0>(angularRate), std::get<1>(angularRate), std::get<2>(angularRate));

    writeDoubleToCSV(x_e(0), x_e(1), x_e(2), v_e(0), v_e(1), v_e(2), 6);

    // Nominal State Calculation //
    x_e += v_e * loopTime + 0.5 * (R * (a_m - a_b) + g) * loopTime * loopTime;
    v_e += (R * (a_m - a_b) + g) * loopTime;

    // Small-angle quaternion
    Eigen::Vector3d theta = (w_m - w_b) * loopTime;
    double angle = theta.norm();

    Eigen::Quaterniond dq;
    if (angle < 1e-12) {
        dq = Eigen::Quaterniond::Identity();
    } else {
        dq = Eigen::AngleAxisd(angle, theta / angle);
    }

    q = (q * dq).normalized();

    // Covariance Calculation //

    Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(15,15);

    Fx.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * loopTime;
    Fx.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    Fx.block<3,3>(3,6) = -R * skew(a_m - a_b) * loopTime;
    Fx.block<3,3>(3,9) = -R * loopTime;
    Fx.block<3,3>(6,6) = dq.toRotationMatrix().transpose();
    Fx.block<3,3>(6,12) = -Eigen::Matrix3d::Identity() * loopTime;
    Fx.block<3,3>(9,9) = Eigen::Matrix3d::Identity();
    Fx.block<3,3>(12,12) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd Fi = Eigen::MatrixXd::Zero(15,12);
    Fi.block<3,3>(3,0) = Eigen::Matrix3d::Identity();
    Fi.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
    Fi.block<3,3>(9,6) = Eigen::Matrix3d::Identity();
    Fi.block<3,3>(12,9) = Eigen::Matrix3d::Identity();

    double sigma_a_n = 0.0316;
    double sigma_w_n = 0.00224;

    Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(12,12);
    Qi.block<3,3>(0,0) = sigma_a_n*sigma_a_n * loopTime*loopTime * Eigen::Matrix3d::Identity();
    Qi.block<3,3>(3,3) = sigma_w_n*sigma_w_n * loopTime*loopTime * Eigen::Matrix3d::Identity();

    // Compute Covariance Matrix //
    P = Fx * P * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Update state estimates with available measurements

    if (std::get<0>(magnetometer.MagnetometerAvailable()) > 0.5) {
        magneticField = magnetometer.GetMagneticField();
        Eigen::Vector3d magneticFieldVector(std::get<0>(magneticField), std::get<1>(magneticField), std::get<2>(magneticField));
        magnetometerUpdate(magneticFieldVector, R);
        // std::cout << "Magnetometer Update: X=" << std::get<0>(magneticField) << ", Y=" << std::get<1>(magneticField) << ", Z=" << std::get<2>(magneticField) << "\n";
    }

    if (std::get<0>(gps.GPSAvailable()) > 0.5) {
        gpsPosition = gps.GetGPSPosition();
        Eigen::Vector3d gpsPositionVector(std::get<0>(gpsPosition), std::get<1>(gpsPosition), std::get<2>(gpsPosition));
        gpsUpdate(gpsPositionVector);
        std::cout << "GPS Update: X=" << std::get<0>(gpsPosition) << ", Y=" << std::get<1>(gpsPosition) << ", Z=" << std::get<2>(gpsPosition) << "\n";
    }

    if (std::get<0>(camera.CameraAvailable()) > 0.5) {
        cameraDirections = camera.GetUnitVectors();
        Eigen::VectorXd cameraDirectionsVector(9);
        cameraDirectionsVector << std::get<0>(cameraDirections), std::get<1>(cameraDirections), std::get<2>(cameraDirections),
                                  std::get<3>(cameraDirections), std::get<4>(cameraDirections), std::get<5>(cameraDirections),
                                  std::get<6>(cameraDirections), std::get<7>(cameraDirections), std::get<8>(cameraDirections);
        cameraUpdate(cameraDirectionsVector, R);
        //std::cout << "Camera Update: X=" << std::get<0>(cameraDirections) << ", Y=" << std::get<1>(cameraDirections) << ", Z=" << std::get<2>(cameraDirections) << "\n";
    }

    // Repack states into stateMat //
    stateMat.segment(0, 3) = x_e;
    stateMat.segment(3, 3) = v_e;
    stateMat(6) = q.w();
    stateMat(7) = q.x();
    stateMat(8) = q.y();
    stateMat(9) = q.z();
    stateMat.segment(10, 3) = a_b;
    stateMat.segment(13, 3) = w_b;
    Eigen::Vector3d angularRateVector = Eigen::Vector3d(std::get<0>(angularRate), std::get<1>(angularRate), std::get<2>(angularRate));
    w = angularRateVector - w_b;
}

void Navigation::magnetometerUpdate(const Eigen::Vector3d& magneticField, const Eigen::Matrix3d& R)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3,3>(0,6) = skew(R.transpose() * MissionConstants::kEarthMagField);
    Eigen::Vector3d y_pred = R.transpose() * MissionConstants::kEarthMagField;
    kalmanUpdate(H, (1e-2) * (1e-2) * Eigen::Matrix3d::Identity(), magneticField, y_pred);
}

void Navigation::gpsUpdate(const Eigen::Vector3d& gpsPosition)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    kalmanUpdate(H, (1) * (1) * Eigen::Matrix3d::Identity(), gpsPosition, x_e);
}

void Navigation::cameraUpdate(const Eigen::VectorXd& cameraDirectionsVector, const Eigen::Matrix3d& R)
{
    int N = MissionConstants::kMarkerData.cols();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3*N, 15);
    Eigen::VectorXd y_pred = Eigen::VectorXd::Zero(3*N);

    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d delta =
            R.transpose() * (MissionConstants::kMarkerData.col(i) - x_e) + MissionConstants::kSensorCameraPosition;

        double d = delta.norm();
        Eigen::Vector3d rho = delta / d;
        Eigen::Vector3d u = (MissionConstants::kMarkerData.col(i) - x_e) / d;
        y_pred.segment<3>(3*i) = rho;

        H.block<3,3>(3*i, 0) =
            -R.transpose() * (Eigen::Matrix3d::Identity() - u*u.transpose()) / d;

        H.block<3,3>(3*i, 6) =
            -skew(R.transpose() * u);
    }
    kalmanUpdate(H, (1e-2) * (1e-2) * Eigen::MatrixXd::Identity(3*N, 3*N), cameraDirectionsVector, y_pred);
}

void Navigation::kalmanUpdate(
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& V,
    const Eigen::VectorXd& y,
    const Eigen::VectorXd& y_pred
) {
    Eigen::VectorXd r = y - y_pred;
    Eigen::MatrixXd S = H * P * H.transpose() + V;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    Eigen::VectorXd dx = K * r;
    P = (Eigen::MatrixXd::Identity(15,15) - K * H) * P;

    // Inject error state
    x_e += dx.segment<3>(0);
    v_e += dx.segment<3>(3);

    Eigen::Vector3d dtheta = dx.segment<3>(6);
    Eigen::Quaterniond dq(1, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
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

Eigen::Matrix3d Navigation::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return S;
}

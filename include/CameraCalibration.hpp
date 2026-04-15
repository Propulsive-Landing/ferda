#pragma once

#include "MissionConstants.hpp"

#include <nlohmann/json.hpp>

#include <opencv2/core.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace CameraCalibration
{
struct Data
{
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;
    cv::Size imageSize;
    std::string cameraId = "default";
    std::string timestampUtc;
    int version = 1;
    int checkerboardColumns = 0;
    int checkerboardRows = 0;
    double checkerboardSquareSizeMeters = 0.0;
    double rmsReprojectionError = -1.0;
};

inline cv::Mat MakeDefaultCameraMatrix()
{
    return (cv::Mat_<double>(3, 3) <<
        MissionConstants::kSensorCameraFocalLengthXPx, 0.0, MissionConstants::kSensorCameraPrincipalPointXPx,
        0.0, MissionConstants::kSensorCameraFocalLengthYPx, MissionConstants::kSensorCameraPrincipalPointYPx,
        0.0, 0.0, 1.0);
}

inline cv::Mat MakeDefaultDistortionCoefficients()
{
    return (cv::Mat_<double>(1, 5) <<
        MissionConstants::kSensorCameraDistortionK1,
        MissionConstants::kSensorCameraDistortionK2,
        MissionConstants::kSensorCameraDistortionP1,
        MissionConstants::kSensorCameraDistortionP2,
        MissionConstants::kSensorCameraDistortionK3);
}

inline Data MakeDefaultCalibration()
{
    Data calibration;
    calibration.cameraMatrix = MakeDefaultCameraMatrix();
    calibration.distortionCoefficients = MakeDefaultDistortionCoefficients();
    calibration.imageSize = cv::Size(
        MissionConstants::kSensorCameraImageWidthPx,
        MissionConstants::kSensorCameraImageHeightPx);
    calibration.timestampUtc = "unknown";
    return calibration;
}

inline std::filesystem::path DefaultCalibrationPath()
{
    return std::filesystem::path(MissionConstants::kSensorCameraCalibrationFilePath);
}

inline std::string CurrentTimestampUtc()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTimeT = std::chrono::system_clock::to_time_t(now);
    std::tm utcTime{};
#if defined(_WIN32)
    gmtime_s(&utcTime, &nowTimeT);
#else
    gmtime_r(&nowTimeT, &utcTime);
#endif

    std::ostringstream timestamp;
    timestamp << std::put_time(&utcTime, "%Y-%m-%dT%H:%M:%SZ");
    return timestamp.str();
}

inline nlohmann::json MatrixToJson(const cv::Mat& matrix)
{
    nlohmann::json rows = nlohmann::json::array();
    for (int row = 0; row < matrix.rows; ++row) {
        nlohmann::json jsonRow = nlohmann::json::array();
        for (int column = 0; column < matrix.cols; ++column) {
            jsonRow.push_back(matrix.at<double>(row, column));
        }
        rows.push_back(jsonRow);
    }
    return rows;
}

inline bool JsonToMatrix(const nlohmann::json& jsonMatrix, cv::Mat& matrix)
{
    if (!jsonMatrix.is_array() || jsonMatrix.empty()) {
        return false;
    }

    const bool isNestedArray = jsonMatrix.front().is_array();
    if (isNestedArray) {
        const int rows = static_cast<int>(jsonMatrix.size());
        const int cols = static_cast<int>(jsonMatrix.front().size());
        if (rows <= 0 || cols <= 0) {
            return false;
        }

        matrix = cv::Mat::zeros(rows, cols, CV_64F);
        for (int row = 0; row < rows; ++row) {
            if (!jsonMatrix[row].is_array() || static_cast<int>(jsonMatrix[row].size()) != cols) {
                return false;
            }
            for (int column = 0; column < cols; ++column) {
                matrix.at<double>(row, column) = jsonMatrix[row][column].get<double>();
            }
        }
        return true;
    }

    matrix = cv::Mat::zeros(1, static_cast<int>(jsonMatrix.size()), CV_64F);
    for (int column = 0; column < static_cast<int>(jsonMatrix.size()); ++column) {
        matrix.at<double>(0, column) = jsonMatrix[column].get<double>();
    }
    return true;
}

inline bool LoadCalibrationFile(const std::filesystem::path& path, Data& calibration, std::string* errorMessage = nullptr)
{
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "unable to open calibration file: " + path.string();
        }
        return false;
    }

    nlohmann::json payload;
    try {
        inputFile >> payload;
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = std::string("failed to parse calibration file: ") + exception.what();
        }
        return false;
    }

    try {
        if (payload.contains("version")) {
            calibration.version = payload["version"].get<int>();
        }
        if (payload.contains("camera_id")) {
            calibration.cameraId = payload["camera_id"].get<std::string>();
        }
        if (payload.contains("timestamp_utc")) {
            calibration.timestampUtc = payload["timestamp_utc"].get<std::string>();
        }
        if (payload.contains("rms_reprojection_error")) {
            calibration.rmsReprojectionError = payload["rms_reprojection_error"].get<double>();
        }
        if (payload.contains("checkerboard") && payload["checkerboard"].is_object()) {
            const auto& checkerboard = payload["checkerboard"];
            if (checkerboard.contains("columns")) {
                calibration.checkerboardColumns = checkerboard["columns"].get<int>();
            }
            if (checkerboard.contains("rows")) {
                calibration.checkerboardRows = checkerboard["rows"].get<int>();
            }
            if (checkerboard.contains("square_size_meters")) {
                calibration.checkerboardSquareSizeMeters = checkerboard["square_size_meters"].get<double>();
            }
        }

        if (payload.contains("image_size") && payload["image_size"].is_object()) {
            const auto& imageSize = payload["image_size"];
            calibration.imageSize = cv::Size(
                imageSize.value("width", MissionConstants::kSensorCameraImageWidthPx),
                imageSize.value("height", MissionConstants::kSensorCameraImageHeightPx));
        }

        if (payload.contains("camera_matrix") && !JsonToMatrix(payload["camera_matrix"], calibration.cameraMatrix)) {
            throw std::runtime_error("invalid camera_matrix field");
        }
        if (payload.contains("distortion_coefficients") && !JsonToMatrix(payload["distortion_coefficients"], calibration.distortionCoefficients)) {
            throw std::runtime_error("invalid distortion_coefficients field");
        }
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = std::string("invalid calibration payload: ") + exception.what();
        }
        return false;
    }

    if (calibration.cameraMatrix.empty()) {
        calibration.cameraMatrix = MakeDefaultCameraMatrix();
    }
    if (calibration.distortionCoefficients.empty()) {
        calibration.distortionCoefficients = MakeDefaultDistortionCoefficients();
    }
    if (calibration.cameraMatrix.rows != 3 || calibration.cameraMatrix.cols != 3) {
        if (errorMessage != nullptr) {
            *errorMessage = "camera_matrix must be 3x3";
        }
        return false;
    }
    if (!(calibration.distortionCoefficients.total() == 5 || calibration.distortionCoefficients.total() == 8 || calibration.distortionCoefficients.total() == 12 || calibration.distortionCoefficients.total() == 14)) {
        if (errorMessage != nullptr) {
            *errorMessage = "distortion_coefficients must have 5, 8, 12, or 14 values";
        }
        return false;
    }

    calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);
    calibration.distortionCoefficients = calibration.distortionCoefficients.reshape(1, 1);
    calibration.distortionCoefficients.convertTo(calibration.distortionCoefficients, CV_64F);
    return true;
}

inline bool SaveCalibrationFile(const std::filesystem::path& path, const Data& calibration, std::string* errorMessage = nullptr)
{
    if (calibration.cameraMatrix.empty() || calibration.cameraMatrix.rows != 3 || calibration.cameraMatrix.cols != 3) {
        if (errorMessage != nullptr) {
            *errorMessage = "camera_matrix must be a 3x3 matrix";
        }
        return false;
    }

    if (calibration.distortionCoefficients.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "distortion_coefficients must not be empty";
        }
        return false;
    }

    std::error_code createDirectoryError;
    const std::filesystem::path parentPath = path.parent_path();
    if (!parentPath.empty()) {
        std::filesystem::create_directories(parentPath, createDirectoryError);
        if (createDirectoryError) {
            if (errorMessage != nullptr) {
                *errorMessage = "failed to create calibration directory: " + createDirectoryError.message();
            }
            return false;
        }
    }

    nlohmann::json payload;
    payload["version"] = calibration.version;
    payload["camera_id"] = calibration.cameraId;
    payload["timestamp_utc"] = calibration.timestampUtc.empty() ? CurrentTimestampUtc() : calibration.timestampUtc;
    payload["rms_reprojection_error"] = calibration.rmsReprojectionError;
    payload["image_size"] = {
        {"width", calibration.imageSize.width},
        {"height", calibration.imageSize.height}
    };
    payload["camera_matrix"] = MatrixToJson(calibration.cameraMatrix);
    payload["distortion_coefficients"] = MatrixToJson(calibration.distortionCoefficients);
    payload["checkerboard"] = {
        {"columns", calibration.checkerboardColumns},
        {"rows", calibration.checkerboardRows},
        {"square_size_meters", calibration.checkerboardSquareSizeMeters}
    };

    std::ofstream outputFile(path);
    if (!outputFile.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "unable to open calibration file for writing: " + path.string();
        }
        return false;
    }

    outputFile << std::setw(2) << payload << std::endl;
    return static_cast<bool>(outputFile);
}
} // namespace CameraCalibration
#include "CameraCalibration.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace
{
void PrintUsage(const char* executableName)
{
    std::cerr << "Usage: " << executableName
              << " <image_dir> <output_file> <checkerboard_columns> <checkerboard_rows> <square_size_meters> [camera_id]"
              << std::endl;
}

bool IsImageFile(const std::filesystem::path& path)
{
    const std::string extension = path.extension().string();
    return extension == ".jpg" || extension == ".jpeg" || extension == ".png" || extension == ".bmp" || extension == ".tif" || extension == ".tiff";
}

std::vector<std::filesystem::path> CollectImages(const std::filesystem::path& imageDirectory)
{
    std::vector<std::filesystem::path> images;
    if (!std::filesystem::exists(imageDirectory)) {
        return images;
    }

    for (const auto& entry : std::filesystem::directory_iterator(imageDirectory)) {
        if (entry.is_regular_file() && IsImageFile(entry.path())) {
            images.push_back(entry.path());
        }
    }

    std::sort(images.begin(), images.end());
    return images;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 6) {
        PrintUsage(argv[0]);
        return 1;
    }

    const std::filesystem::path imageDirectory = argv[1];
    const std::filesystem::path outputFile = argv[2];

    int checkerboardColumns = 0;
    int checkerboardRows = 0;
    double squareSizeMeters = 0.0;

    try {
        checkerboardColumns = std::stoi(argv[3]);
        checkerboardRows = std::stoi(argv[4]);
        squareSizeMeters = std::stod(argv[5]);
    } catch (const std::exception& exception) {
        std::cerr << "Failed to parse calibration arguments: " << exception.what() << std::endl;
        PrintUsage(argv[0]);
        return 1;
    }

    if (checkerboardColumns <= 0 || checkerboardRows <= 0 || squareSizeMeters <= 0.0) {
        std::cerr << "Checkerboard dimensions and square size must be positive." << std::endl;
        return 1;
    }

    CameraCalibration::Data calibration = CameraCalibration::MakeDefaultCalibration();
    calibration.cameraId = argc >= 7 ? argv[6] : "offline-image-set";
    calibration.version = 1;
    calibration.checkerboardColumns = checkerboardColumns;
    calibration.checkerboardRows = checkerboardRows;
    calibration.checkerboardSquareSizeMeters = squareSizeMeters;

    const std::vector<std::filesystem::path> images = CollectImages(imageDirectory);
    if (images.empty()) {
        std::cerr << "No calibration images found in " << imageDirectory.string() << std::endl;
        return 1;
    }

    const cv::Size patternSize(checkerboardColumns, checkerboardRows);
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point3f> objectPointTemplate;
    objectPointTemplate.reserve(static_cast<size_t>(checkerboardColumns * checkerboardRows));
    for (int row = 0; row < checkerboardRows; ++row) {
        for (int column = 0; column < checkerboardColumns; ++column) {
            objectPointTemplate.emplace_back(
                static_cast<float>(column * squareSizeMeters),
                static_cast<float>(row * squareSizeMeters),
                0.0f);
        }
    }

    cv::Size imageSize;
    size_t usableImageCount = 0;
    for (const auto& imagePath : images) {
        cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Skipping unreadable image: " << imagePath.string() << std::endl;
            continue;
        }

        if (imageSize.width == 0 || imageSize.height == 0) {
            imageSize = image.size();
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        const bool found = cv::findChessboardCorners(
            gray,
            patternSize,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (!found) {
            std::cerr << "Checkerboard not found in: " << imagePath.string() << std::endl;
            continue;
        }

        cv::cornerSubPix(
            gray,
            corners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));

        objectPoints.push_back(objectPointTemplate);
        imagePoints.push_back(corners);
        ++usableImageCount;
        std::cerr << "Accepted calibration image: " << imagePath.string() << std::endl;
    }

    if (usableImageCount < 3) {
        std::cerr << "Need at least 3 usable calibration images; found " << usableImageCount << std::endl;
        return 1;
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    const double rmsError = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distortionCoefficients,
        rvecs,
        tvecs);

    calibration.cameraMatrix = cameraMatrix;
    calibration.distortionCoefficients = distortionCoefficients;
    calibration.imageSize = imageSize;
    calibration.rmsReprojectionError = rmsError;
    calibration.timestampUtc = CameraCalibration::CurrentTimestampUtc();

    std::string errorMessage;
    if (!CameraCalibration::SaveCalibrationFile(outputFile, calibration, &errorMessage)) {
        std::cerr << "Failed to save calibration: " << errorMessage << std::endl;
        return 1;
    }

    std::cout << "Calibration complete. Saved " << outputFile.string() << std::endl;
    std::cout << "Usable images: " << usableImageCount << std::endl;
    std::cout << "RMS reprojection error: " << rmsError << std::endl;
    return 0;
}
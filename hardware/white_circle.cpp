#include "WhiteCircle.hpp"

#include <algorithm>

namespace WhiteCircle
{
std::vector<MarkerDetection> DetectWhiteMarkerCentroids(
    const cv::Mat& bgrImage,
    int minAreaPx,
    int maxMarkers)
{
    if (bgrImage.empty() || maxMarkers <= 0) {
        return {};
    }

    cv::Mat gray;
    cv::cvtColor(bgrImage, gray, cv::COLOR_BGR2GRAY);

    cv::threshold(gray, gray, 200, 255, cv::THRESH_BINARY);

    cv::Mat channels[3];
    cv::split(bgrImage, channels);

    cv::Mat redMask;
    cv::Mat blueMask;
    cv::Mat greenMask;
    cv::compare(channels[2], 125, redMask, cv::CMP_GT);
    cv::compare(channels[0], 175, blueMask, cv::CMP_GT);
    cv::compare(channels[1], 175, greenMask, cv::CMP_GT);

    cv::Mat colorMask;
    cv::bitwise_and(redMask, blueMask, colorMask);
    cv::bitwise_and(colorMask, greenMask, colorMask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int numLabels = cv::connectedComponentsWithStats(
        colorMask,
        labels,
        stats,
        centroids,
        8);

    std::vector<MarkerDetection> detections;
    detections.reserve(static_cast<size_t>(maxMarkers));

    for (int i = 1; i < numLabels; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < minAreaPx) {
            continue;
        }

        MarkerDetection detection;
        detection.areaPx = area;
        detection.centroidPx.x = centroids.at<double>(i, 0);
        detection.centroidPx.y = centroids.at<double>(i, 1);
        detections.push_back(detection);
    }

    std::sort(
        detections.begin(),
        detections.end(),
        [](const MarkerDetection& a, const MarkerDetection& b) {
            return a.areaPx > b.areaPx;
        });

    if (static_cast<int>(detections.size()) > maxMarkers) {
        detections.resize(static_cast<size_t>(maxMarkers));
    }

    return detections;
}
}

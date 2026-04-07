#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace WhiteCircle
{
struct MarkerDetection
{
    cv::Point2d centroidPx;
    int areaPx = 0;
};

std::vector<MarkerDetection> DetectWhiteMarkerCentroids(
    const cv::Mat& bgrImage,
    int minAreaPx,
    int maxMarkers);
}

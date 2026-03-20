#pragma once

#include <tuple>

class Lidar
{
public:
    Lidar();
    std::tuple<double> GetLidarDistance();
    std::tuple<double> LidarAvailable();
};

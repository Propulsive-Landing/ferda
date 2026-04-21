#pragma once

#include <tuple>

class Lidar
{
public:
    Lidar();
    std::tuple<double> GetLidarDistance();
    bool GetUseLidar() { return useLidar; }
    void SetUseLidar(const bool &state) { useLidar = state; }

private:
    bool useLidar;
};

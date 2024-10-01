#pragma once

#include <Eigen/Dense>

#include "MissionConstants.hpp"

class TVC
{
    private:


        Eigen::Matrix<double, 2, 1> TVC_SERVO_HORN_OFFSET = (Eigen::MatrixXd(2,1) << 
                                101.0L, 
                                100.0L).finished();

        Eigen::Matrix<double, 2, 2> TVC_GEAR_RATIO = (Eigen::MatrixXd(2,2) << 
                                -1.0L / 0.317705L, 0.0L, 
                                0.0L, -1.0L / 0.31731096L ).finished();

  

    public:
        TVC() = default;
        void SetTVCX(double angle_rad); 
        void SetTVCY(double angle_rad); 
};
#pragma once

#if NDEBUG
#include <memory>
#include <PiPCA9685/PCA9685.h>
inline std::unique_ptr<PiPCA9685::PCA9685> servo_driver;
#endif

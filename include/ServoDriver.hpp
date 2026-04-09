#pragma once

#if NDEBUG
#include <PiPCA9685/PCA9685.h>
PiPCA9685::PCA9685 servo_driver;
servo_driver.set_pwm_freq(50);
#endif

#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include "Telemetry.hpp"
#include "RF.hpp"
#include "LinActMotorPositionControl.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <iostream>
#include <thread>

namespace
{
void LogActuatorCalibrationInstructions()
{
    Telemetry::GetInstance().Log("ACTUATOR CALIBRATION MODE");
    Telemetry::GetInstance().Log("WARNING: Actuators should NOT be connected to the TVC linkage in this mode.");
    Telemetry::GetInstance().Log("Commands: MoveXTVCToLimitExtend, MoveXTVCToLimitRetract, MoveYTVCToLimitExtend, MoveYTVCToLimitRetract");
    Telemetry::GetInstance().Log("Use StopTVC to halt motion, CenterTVC to center, and GoIdle to exit.");
}
}

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

void Mode::UploadKmatrices()
{
    AngleKMatrix = "../Angles.csv";
    HeightKMatrix = "../Height.csv";
    TranslationKMatrix = "../Translation.csv";
}

Mode::Phase Mode::UpdateCalibration(Navigation &navigation, Controller &controller, double currentTime)
{
    static float XTVC = 0.0;
    static float YTVC = 0.0;
    RF::Command command = RF::GetInstance().GetCommand();
    if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in calibration");
        controller.tvc.Stop();
        return Mode::Calibration;
    }
    if (command == RF::Command::CenterTVC)
    {
        Telemetry::GetInstance().Log("CENTER TVC command received in calibration");
        XTVC = 0.0;
        YTVC = 0.0;
        controller.Center();
        return Mode::Calibration;
    }
    if (command == RF::Command::ActuatorCalibration)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to actuator calibration");
        LogActuatorCalibrationInstructions();
        controller.Center();
        return Mode::ActuatorCalibration;
    }
    if (command == RF::Command::ChirpTVC)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to chirp tvc");
        return Mode::ChirpTVC;
    }
    if (command == RF::Command::IncrementXTVC)
    {
        XTVC += 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCX(XTVC);
        return Mode::Calibration;
    }
    if (command == RF::Command::IncrementYTVC)
    {
        YTVC += 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCY(YTVC);
        return Mode::Calibration;
    }
    if (command == RF::Command::DecrementXTVC)
    {
        XTVC -= 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCX(XTVC);
        return Mode::Calibration;
    }
    if (command == RF::Command::DecrementYTVC)
    {
        YTVC -= 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCY(YTVC);
        return Mode::Calibration;
    }
    if (command == RF::Command::TestTVC)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to test tvc");
        UploadKmatrices();
        controller.ImportAngleParameters(AngleKMatrix);
        controller.ImportHeightParameters(HeightKMatrix);
        controller.ImportTranslationParameters(TranslationKMatrix);
        controller.Center();
        return Mode::TestTVC;
    }
    else if (command == RF::Command::GoIdle)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to idle");
        UploadKmatrices();
        controller.ImportAngleParameters(AngleKMatrix);
        controller.ImportHeightParameters(HeightKMatrix);
        controller.ImportTranslationParameters(TranslationKMatrix);
        controller.Center();
        return Mode::Idle;
    }
    else if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        exit(0);
    }

    controller.tvc.UpdateActuatorPositions();
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateActuatorCalibration(Navigation &navigation, Controller &controller, double currentTime)
{
    navigation.UpdateNavigation();

    RF::Command command = RF::GetInstance().GetCommand();
    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        exit(0);
    }
    else if (command == RF::Command::GoIdle)
    {
        Telemetry::GetInstance().Log("Leaving actuator calibration for idle");
        controller.tvc.Stop();
        return Mode::Idle;
    }
    else if (command == RF::Command::CenterTVC)
    {
        Telemetry::GetInstance().Log("CENTER TVC command received in actuator calibration");
        controller.Center();
        return Mode::ActuatorCalibration;
    }
    else if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in actuator calibration");
        controller.tvc.Stop();
        return Mode::ActuatorCalibration;
    }
    else if (command == RF::Command::MoveXTVCToLimitExtend)
    {
        Telemetry::GetInstance().Log("MOVE X TVC TO LIMIT (extend) received in actuator calibration");
        moveToLimit(0, 1);
        controller.tvc.Stop();
        return Mode::ActuatorCalibration;
    }
    else if (command == RF::Command::MoveXTVCToLimitRetract)
    {
        Telemetry::GetInstance().Log("MOVE X TVC TO LIMIT (retract) received in actuator calibration");
        moveToLimit(0, -1);
        controller.tvc.Stop();
        return Mode::ActuatorCalibration;
    }
    else if (command == RF::Command::MoveYTVCToLimitExtend)
    {
        Telemetry::GetInstance().Log("MOVE Y TVC TO LIMIT (extend) received in actuator calibration");
        moveToLimit(1, 1);
        controller.tvc.Stop();
        return Mode::ActuatorCalibration;
    }
    else if (command == RF::Command::MoveYTVCToLimitRetract)
    {
        Telemetry::GetInstance().Log("MOVE Y TVC TO LIMIT (retract) received in actuator calibration");
        moveToLimit(1, -1);
        controller.tvc.Stop();
        return Mode::ActuatorCalibration;
    }

    controller.tvc.UpdateActuatorPositions();
    return Mode::ActuatorCalibration;
}

Mode::Phase Mode::UpdateTestTVC(Navigation &navigation, Controller &controller, double currentTime)
{

    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;

    controller.UpdateTestTVC(seconds_since_start);

    RF::Command command = RF::GetInstance().GetCommand();
    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        exit(0);
    }
    else if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in test mode");
        controller.tvc.Stop();
        return Mode::Idle;
    }
    else if (command == RF::Command::CenterTVC)
    {
        Telemetry::GetInstance().Log("CENTER TVC command received in test mode");
        controller.Center();
        return Mode::Idle;
    }
    else if (seconds_since_start >= 10)
    {
        Telemetry::GetInstance().Log("Switching mode from test to idle");
        controller.Center();
        return Mode::Idle;
    }

    return Mode::TestTVC;
}

Mode::Phase Mode::UpdateChirpTVC(Navigation &navigation, Controller &controller, double currentTime)
{
    Telemetry::GetInstance().Log("Starting fixed-parameter chirp TVC test");
    RunChirpTVCMode();
    Telemetry::GetInstance().Log("Finished chirp TVC test");
    controller.Center();
    return Mode::Idle;
}

Mode::Phase Mode::UpdateIdle(Navigation &navigation, Controller &controller, IMU &imu, double currentTime)
{
    // Enable pad updates while on the pad
    navigation.SetOnPad(true);

    navigation.UpdateNavigation();

    // launch when we get the command
    RF::Command command = RF::GetInstance().GetCommand();
    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        exit(0);
    }
    else if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in idle");
        controller.tvc.Stop();
        return Mode::Idle;
    }
    else if (command == RF::Command::CenterTVC)
    {
        Telemetry::GetInstance().Log("CENTER TVC command received in idle");
        controller.Center();
        return Mode::Idle;
    }
    else if (command == RF::Command::Ignite)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        navigation.SetOnPad(false); // Disable pad updates during flight
        navigation.reset();
        return Mode::Launch;
    }
    else if (command == RF::Command::ChirpTVC)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to chirp tvc");
        return Mode::ChirpTVC;
    }

    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(Navigation &navigation, Controller &controller, Igniter &igniter, double currentTime)
{
    // Manage ignition and controller start on first call, then delegate launch behavior
    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;
    static int startup = 1;

    if (startup == 1)
    {
        Telemetry::GetInstance().Log("Igniting LAUNCH MOTOR");
        igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);
        controller.Start(seconds_since_start);
        startup = 0;
        // reset launch manager
        this->launchManager.Reset();
    }

    if (seconds_since_start > 0.050)
    {
        igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAUNCH);
    }

    bool handoffToSafe = this->launchManager.Step(navigation, controller, igniter, currentTime);
    if (handoffToSafe)
    {
        return Mode::Safe;
    }

    return Mode::Launch;
}

Mode::Phase Mode::UpdateSafeMode(Navigation &navigation, Controller &controller, double currentTime)
{
    // continue collection data
    navigation.UpdateNavigation();
    controller.UpdateSafe();

    return Mode::Terminate;
}

bool Mode::Update(Navigation &navigation, Controller &controller, GPS &gps, Igniter &igniter, IMU &imu)
{

    // Track total elapsed time and delta time
    static double currentTime = 0;
    static auto last_time = std::chrono::high_resolution_clock::now();
    // Helpful when running SIL
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    auto time_now = std::chrono::high_resolution_clock::now();
    long long nanoseconds_since_start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - last_time).count();
    double change_time = nanoseconds_since_start / 1000000000.0;
    last_time = time_now;

    currentTime += change_time;

    // Update navigations and controller's loopTime to be changeTime which should be 0.005 each time
    navigation.loopTime = change_time;
    controller.loopTime = change_time;

    /* Handle behavior based on current phase. Update phase*/
    switch (this->eCurrentMode)
    {
    case Calibration:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, 0.05, 0.08);
        this->eCurrentMode = UpdateCalibration(navigation, controller, currentTime);
        break;
    case ActuatorCalibration:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, 0.05, 0.08);
        this->eCurrentMode = UpdateActuatorCalibration(navigation, controller, currentTime);
        break;
    case TestTVC:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, 0.05, 0.08);
        this->eCurrentMode = UpdateTestTVC(navigation, controller, currentTime);
        break;
    case ChirpTVC:
        this->eCurrentMode = UpdateChirpTVC(navigation, controller, currentTime);
        break;
    case Idle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, 0.05, 0.08);
        this->eCurrentMode = UpdateIdle(navigation, controller, imu, currentTime);
        break;
    case Launch:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, 0.01, 0.08);
        this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, currentTime);
        break;
    case Safe:
        this->eCurrentMode = UpdateSafeMode(navigation, controller, currentTime);
        break;
    case Terminate:
        return false;
    }

    return true;
}

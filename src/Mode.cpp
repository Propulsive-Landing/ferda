#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include "Telemetry.hpp"
#include "ValveControl.hpp"
#include "SparkPlug.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"
#include "GPS.hpp"
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
        Telemetry::GetInstance().Log("Use StopTVC to halt motion, CenterTVC to center, and Idle to exit.");
    }
}

void Mode::CloseAllValvesAndSparkPlug(ValveControl &valveControl, SparkPlug &sparkPlug)
{

    valveControl.CloseValve(ValveControl::ASIEthanol);
    valveControl.CloseValve(ValveControl::ASIOxygen);
    valveControl.OpenValve(ValveControl::NitrogenBleed);
    sparkPlug.TurnOff();

    valveControl.CloseValve(ValveControl::Nitrogen);
    valveControl.CloseValve(ValveControl::Purge);
    valveControl.CloseValve(ValveControl::MainEthanol);
    valveControl.CloseValve(ValveControl::MainNitrous);
    valveControl.CloseValve(ValveControl::NitrousFill);
}

Mode::Phase Mode::UpdateStandby(RF::Command &command, Controller &controller)
{
    // Stop TVC
    controller.tvc.Stop();

    if (command == RF::Calibration)
    {
        Telemetry::GetInstance().Log("Switching mode from standby to calibration");
        return Mode::Calibration;
    }
    if (command == RF::Idle)
    {
        Telemetry::GetInstance().Log("Switching mode from standby to Idle");
        return Mode::Idle;
    }

    return Mode::Standby;
}

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

void Mode::UploadKmatrices()
{
    AngleKMatrix = "../Angles.csv";
    HeightKMatrix = "../Height.csv";
    TranslationKMatrix = "../Translation.csv";
}

void Mode::CheckForToggleSensorCommands(RF::Command &command, GPS &gps, Camera &camera,
                                        Magnetometer &magnetometer)
{
    if (command == RF::Command::CameraOn)
    {
        Telemetry::GetInstance().Log("Switching Camera on");
        camera.setUseCamera(true);
    }
    else if (command == RF::Command::CameraOff)
    {
        Telemetry::GetInstance().Log("Switching Camera off");
        camera.setUseCamera(false);
    }
    else if (command == RF::Command::GPSPositionOn)
    {
        Telemetry::GetInstance().Log("Switching GPS Position on");
        gps.SetUseGPSPosition(true);
    }
    else if (command == RF::Command::GPSPositionOff)
    {
        Telemetry::GetInstance().Log("Switching GPS Position off");
        gps.SetUseGPSPosition(false);
    }
    else if (command == RF::Command::GPSVelocityOn)
    {
        Telemetry::GetInstance().Log("Switching GPS Velocity on");
        gps.SetUseGPSVelocity(true);
    }
    else if (command == RF::Command::GPSVelocityOff)
    {
        Telemetry::GetInstance().Log("Switching GPS Velocity off");
        gps.SetUseGPSVelocity(false);
    }
    else if (command == RF::Command::MagnetometerOn)
    {
        Telemetry::GetInstance().Log("Switching Magnometer on");
        magnetometer.setUseMagnometer(true);
    }
    else if (command == RF::Command::MagnetometerOff)
    {
        Telemetry::GetInstance().Log("Switching Magnometer off");
        magnetometer.setUseMagnometer(false);
    }
}

void Mode::CheckForToggleValveCommands(RF::Command &command, ValveControl &valveControl, SparkPlug &sparkPlug)
{
    if (command == RF::Command::ValveNitrogenOpen)
    {
        valveControl.OpenValve(ValveControl::Nitrogen);
    }
    else if (command == RF::Command::ValveNitrogenClose)
    {
        valveControl.CloseValve(ValveControl::Nitrogen);
    }
    else if (command == RF::Command::ValvePurgeOpen)
    {
        valveControl.OpenValve(ValveControl::Purge);
    }
    else if (command == RF::Command::ValvePurgeClose)
    {
        valveControl.CloseValve(ValveControl::Purge);
    }
    else if (command == RF::Command::ValveMainEthanolOpen)
    {
        valveControl.OpenValve(ValveControl::MainEthanol);
    }
    else if (command == RF::Command::ValveMainEthanolClose)
    {
        valveControl.CloseValve(ValveControl::MainEthanol);
    }
    else if (command == RF::Command::ValveMainNitrousOpen)
    {
        valveControl.OpenValve(ValveControl::MainNitrous);
    }
    else if (command == RF::Command::ValveMainNitrousClose)
    {
        valveControl.CloseValve(ValveControl::MainNitrous);
    }
    else if (command == RF::Command::ValveNitrousFillOpen)
    {
        valveControl.OpenValve(ValveControl::NitrousFill);
    }
    else if (command == RF::Command::ValveNitrousFillClose)
    {
        valveControl.CloseValve(ValveControl::NitrousFill);
    }
    else if (command == RF::Command::ValveASIEthanolOpen)
    {
        valveControl.OpenValve(ValveControl::ASIEthanol);
    }
    else if (command == RF::Command::ValveASIEthanolClose)
    {
        valveControl.CloseValve(ValveControl::ASIEthanol);
    }
    else if (command == RF::Command::ValveASIOxygenOpen)
    {
        valveControl.OpenValve(ValveControl::ASIOxygen);
    }
    else if (command == RF::Command::ValveASIOxygenClose)
    {
        valveControl.CloseValve(ValveControl::ASIOxygen);
    }
    else if (command == RF::Command::ValveNitrogenBleedOpen)
    {
        valveControl.OpenValve(ValveControl::NitrogenBleed);
    }
    else if (command == RF::Command::ValveNitrogenBleedClose)
    {
        valveControl.CloseValve(ValveControl::NitrogenBleed);
    }
    // Handle spark commands
    else if (command == RF::Command::SparkOn)
    {
        sparkPlug.TurnOn();
    }
    else if (command == RF::Command::SparkOff)
    {
        sparkPlug.TurnOff();
    }
}

Mode::Phase Mode::UpdateCalibration(RF::Command &command, Navigation &navigation, Controller &controller,
                                    double currentTime)
{
    static float XTVC = 0.0;
    static float YTVC = 0.0;

    if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in calibration");
        Telemetry::GetInstance().Log("Going to standby, EXITING");
        return Mode::Standby;
    }
    else if (command == RF::Command::Standby)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to Standby");
        return Mode::Standby;
    }
    else if (command == RF::Command::CenterTVC)
    {
        Telemetry::GetInstance().Log("CENTER TVC command received in calibration");
        XTVC = 0.0;
        YTVC = 0.0;
        controller.Center();
        return Mode::Calibration;
    }
    else if (command == RF::Command::ActuatorCalibration)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to actuator calibration");
        LogActuatorCalibrationInstructions();
        controller.Center();
        return Mode::ActuatorCalibration;
    }
    else if (command == RF::Command::ChirpTVC)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to chirp tvc");
        return Mode::ChirpTVC;
    }
    else if (command == RF::Command::IncrementXTVC)
    {
        XTVC += 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCX(XTVC);
        return Mode::Calibration;
    }
    else if (command == RF::Command::IncrementYTVC)
    {
        YTVC += 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCY(YTVC);
        return Mode::Calibration;
    }
    else if (command == RF::Command::DecrementXTVC)
    {
        XTVC -= 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCX(XTVC);
        return Mode::Calibration;
    }
    else if (command == RF::Command::DecrementYTVC)
    {
        YTVC -= 0.01;
        std::ostringstream os;
        os << "TVC Position, X: " << std::to_string(XTVC) << " Y: " << std::to_string(YTVC) << std::endl;
        std::string s = os.str();
        Telemetry::GetInstance().Log(s);
        controller.tvc.SetTVCY(YTVC);
        return Mode::Calibration;
    }
    else if (command == RF::Command::TestTVC)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to test tvc");
        UploadKmatrices();
        controller.ImportAngleParameters(AngleKMatrix);
        controller.ImportHeightParameters(HeightKMatrix);
        controller.ImportTranslationParameters(TranslationKMatrix);
        controller.Center();
        return Mode::TestTVC;
    }
    else if (command == RF::Command::Idle)
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
        return Mode::Abort;
    }

    controller.tvc.UpdateActuatorPositions();
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateActuatorCalibration(RF::Command &command, Navigation &navigation, Controller &controller, double currentTime)
{
    navigation.UpdateNavigation();

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        return Mode::Abort;
    }
    else if (command == RF::Command::Standby)
    {
        Telemetry::GetInstance().Log("Switching mode from updateActuatorCalibration to Standby");
        return Mode::Standby;
    }
    else if (command == RF::Command::Idle)
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
        return Mode::Standby;
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

Mode::Phase Mode::UpdateTestTVC(RF::Command &command, Navigation &navigation, Controller &controller,
                                double currentTime)
{
    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;

    controller.UpdateTestTVC(seconds_since_start);

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        return Mode::Abort;
    }
    else if (command == RF::Command::Standby)
    {
        Telemetry::GetInstance().Log("Switching mode from testTVC to Standby");
        return Mode::Standby;
    }
    else if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in test mode");
        return Mode::Standby;
    }
    else if (command == RF::Command::CenterTVC)
    {
        Telemetry::GetInstance().Log("CENTER TVC command received in test mode");
        controller.Center();
        return Mode::Calibration;
    }
    else if (seconds_since_start >= 10)
    {
        Telemetry::GetInstance().Log("Switching mode from test to calibration");
        controller.Center();
        // Command both Actuators to stop
        driveActuator(0, 0, 0);
        driveActuator(1, 0, 0);
        return Mode::Calibration;
    }

    return Mode::TestTVC;
}

Mode::Phase Mode::UpdateChirpTVC(Navigation &navigation, Controller &controller, double currentTime)
{
    Telemetry::GetInstance().Log("Starting fixed-parameter chirp TVC test");
    RunChirpTVCMode();
    controller.Center();
    Telemetry::GetInstance().Log("Switching mode from chirp tvc to calibration");
    return Mode::Calibration;
}

Mode::Phase Mode::UpdateIdle(RF::Command &command, Navigation &navigation, Controller &controller,
                             double currentTime)
{
    // Enable pad updates while on the pad
    navigation.SetOnPad(true);

    navigation.UpdateNavigation();

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        return Mode::Abort;
    }
    else if (command == RF::Command::Standby)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to Standby");
        return Mode::Standby;
    }
    else if (command == RF::Command::StopTVC)
    {
        Telemetry::GetInstance().Log("STOP TVC command received in idle");
        return Mode::Standby;
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
    else if (command == RF::Command::HotfireIdle)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to HotfireIdle");
        return Mode::HotfireIdle;
    }
    else if (command == RF::Command::ChirpTVC)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to chirp tvc");
        return Mode::ChirpTVC;
    }
    else if (command == RF::Command::NAV_RESTART)
    {
        Telemetry::GetInstance().Log("NAV_RESTART command received, resetting navigation");
        navigation.hard_reset();
        command = RF::Command::None;
    }

    return Mode::Idle;
}

Mode::Phase Mode::UpdateLaunch(RF::Command &command, Navigation &navigation, Controller &controller, Igniter &igniter, float currentTime)
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

    bool handoffToSafe = this->launchManager.Step(command, navigation, controller, igniter, currentTime);
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

// Liquid Propulsion State Implementations
Mode::Phase Mode::UpdateHotfireIdle(RF::Command &command, Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug)
{
    navigation.UpdateNavigation();

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        return Mode::Abort;
    }
    else if (command == RF::Command::Standby)
    {
        Telemetry::GetInstance().Log("Switching mode from HotfireIdle to Standby");
        return Mode::Standby;
    }
    else if (command == RF::Command::ASITest)
    {
        Telemetry::GetInstance().Log("Switching mode from HotfireIdle to ASITest");
        return Mode::ASITest;
    }
    else if (command == RF::Command::WaterFlow)
    {
        Telemetry::GetInstance().Log("Switching mode from HotfireIdle to WaterFlow");
        return Mode::WaterFlow;
    }
    else if (command == RF::Command::ThreeSecondHotfire)
    {
        Telemetry::GetInstance().Log("Switching mode from HotfireIdle to ThreeSecondHotfire");
        return Mode::ThreeSecondHotfire;
    }
    else if (command == RF::Command::Idle)
    {
        Telemetry::GetInstance().Log("Switching mode from HotfireIdle to Idle");
        return Mode::Idle;
    }

    return Mode::HotfireIdle;
}

Mode::Phase Mode::UpdateASITest(RF::Command &command, Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug,
                                double currentTime)
{
    static double startTime;
    static bool sequenceStarted = false;
    static bool firstPartDone = false;
    static bool secondPartDone = false;
    static bool thirdPartDone = false;

    if (!sequenceStarted)
    {
        startTime = currentTime;
        Telemetry::GetInstance().Log("Starting ASI Test sequence");
        valveControl.OpenValve(ValveControl::ASIOxygen);
        sparkPlug.TurnOn();
        sequenceStarted = true;
    }

    double seconds_since_start = currentTime - startTime;
    navigation.UpdateNavigation();

    if (seconds_since_start >= 0.3 && seconds_since_start < 2.3)
    {
        // Open ASI ethanol
        if (!firstPartDone)
        {
            // Uncomment for debugging
            // std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.OpenValve(ValveControl::ASIEthanol);
            firstPartDone = true;
        }
    }
    else if (seconds_since_start >= 2.3 && seconds_since_start < 2.6)
    {
        // Close ASI ethanol and turn off spark plug
        if (!secondPartDone)
        {
            // Uncomment for debugging
            // std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.CloseValve(ValveControl::ASIEthanol);
            sparkPlug.TurnOff();
            secondPartDone = true;
        }
    }
    else if (seconds_since_start >= 2.6)
    {
        // Close ASI oxygen
        if (!thirdPartDone)
        {
            // Uncomment for debugging
            // std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.CloseValve(ValveControl::ASIOxygen);
            thirdPartDone = true;
        }
        // Return to HotfireIdle after sequence completes
        if (seconds_since_start >= 3.0)
        {
            Telemetry::GetInstance().Log("asitest sequence completed, returning to HotfireIdle");
            sequenceStarted = false;
            firstPartDone = false;
            secondPartDone = false;
            thirdPartDone = false;
            return Mode::HotfireIdle;
        }
    }
    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT during ASI Test, returning to HotfireIdle");
        // Close all valves and turn off spark
        valveControl.CloseValve(ValveControl::ASIOxygen);
        valveControl.CloseValve(ValveControl::ASIEthanol);
        sparkPlug.TurnOff();
        return Mode::HotfireIdle;
    }

    return Mode::ASITest;
}

Mode::Phase Mode::UpdateWaterFlow(RF::Command &command, Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug,
                                  double currentTime)
{
    static double startTime;
    static bool sequenceStarted = false;
    static bool firstPartDone = false;
    static bool secondPartDone = false;

    if (!sequenceStarted)
    {
        startTime = currentTime;
        Telemetry::GetInstance().Log("Starting Water Flow sequence");
        valveControl.OpenValve(ValveControl::MainNitrous);
        sequenceStarted = true;
    }

    double seconds_since_start = currentTime - startTime;
    navigation.UpdateNavigation();

    if (seconds_since_start >= 1.0 && seconds_since_start < 2.0)
    {
        // Open main ethanol
        if (!firstPartDone)
        {
            // Uncomment for debugging
            // std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.OpenValve(ValveControl::MainEthanol);
            firstPartDone = true;
        }
    }
    else if (seconds_since_start >= 3.0)
    {
        // Close main nitrous and main ethanol valves
        if (!secondPartDone)
        {
            // Uncomment for debugging
            // std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.CloseValve(ValveControl::MainNitrous);
            valveControl.CloseValve(ValveControl::MainEthanol);
            secondPartDone = true;
        }
        // Return to HotfireIdle after sequence completes
        if (seconds_since_start >= 3.5)
        {
            Telemetry::GetInstance().Log("Water Flow sequence completed, returning to HotfireIdle");
            sequenceStarted = false;
            firstPartDone = false;
            secondPartDone = false;
            return Mode::HotfireIdle;
        }
    }

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT during Water Flow, returning to HotfireIdle");
        // Close all valves
        valveControl.CloseValve(ValveControl::MainNitrous);
        valveControl.CloseValve(ValveControl::MainEthanol);
        return Mode::HotfireIdle;
    }

    return Mode::WaterFlow;
}

Mode::Phase Mode::Update3SecondHotfire(RF::Command &command,
                                       Navigation &navigation,
                                       Controller &controller,
                                       ValveControl &valveControl,
                                       SparkPlug &sparkPlug,
                                       double currentTime)
{
    static double startTime;
    static bool sequenceStarted = false;

    static bool firstPartDone = false;
    static bool secondPartDone = false;
    static bool thirdPartDone = false;
    static bool fourthPartDone = false;
    static bool fifthPartDone = false;
    static bool sixthPartDone = false;

    if (!sequenceStarted)
    {
        startTime = currentTime;

        Telemetry::GetInstance().Log("Starting 3 Second Hotfire sequence");

        // Turn on ASI oxygen and spark plug
        valveControl.OpenValve(ValveControl::ASIOxygen);
        sparkPlug.TurnOn();

        sequenceStarted = true;
    }

    double seconds_since_start = currentTime - startTime;

    navigation.UpdateNavigation();

    if (seconds_since_start >= 0.2 && seconds_since_start < 0.5)
    {
        // Turn on ASI ethanol
        if (!firstPartDone)
        {
            std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.OpenValve(ValveControl::ASIEthanol);

            firstPartDone = true;
        }
    }
    else if (seconds_since_start >= 0.5 && seconds_since_start < 0.7)
    {
        // Open main nitrous and main ethanol  valve
        if (!secondPartDone)
        {
            std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.OpenValve(ValveControl::MainNitrous);
            valveControl.OpenValve(ValveControl::MainEthanol);
            secondPartDone = true;
        }
    }
    else if (seconds_since_start >= 0.7 && seconds_since_start < 3.4)
    {
        // Turn off spark plug and ASI oxygen
        if (!thirdPartDone)
        {
            std::cout << "Time: " << seconds_since_start << "\n";

            sparkPlug.TurnOff();
            valveControl.CloseValve(ValveControl::ASIOxygen);

            thirdPartDone = true;
        }

        controller.UpdateTestTVC(seconds_since_start);
    }
    else if (seconds_since_start >= 3.4 && seconds_since_start < 3.6)
    {
        // Turn off TVC, ASI ethanol, main ethanol
        if (!fourthPartDone)
        {
            std::cout << "Time: " << seconds_since_start << "\n";

            controller.tvc.Stop();

            valveControl.CloseValve(ValveControl::ASIEthanol);
            valveControl.CloseValve(ValveControl::MainEthanol);

            fourthPartDone = true;
        }
    }
    else if (seconds_since_start >= 3.6 && seconds_since_start < 4.1)
    {
        // Close main nitrous
        if (!fifthPartDone)
        {
            std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.CloseValve(ValveControl::MainNitrous);

            fifthPartDone = true;
        }
    }
    else if (seconds_since_start >= 4.1 && seconds_since_start < 5.6)
    {
        // Open purge
        if (!sixthPartDone)
        {
            std::cout << "Time: " << seconds_since_start << "\n";

            valveControl.OpenValve(ValveControl::Purge);

            sixthPartDone = true;
        }
    }
    else if (seconds_since_start >= 5.6)
    {
        // Close purge
        std::cout << "Time: " << seconds_since_start << "\n";

        valveControl.CloseValve(ValveControl::Purge);

        Telemetry::GetInstance().Log(
            "3 Second Hotfire sequence completed, returning to HotfireIdle");

        sequenceStarted = false;

        firstPartDone = false;
        secondPartDone = false;
        thirdPartDone = false;
        fourthPartDone = false;
        fifthPartDone = false;
        sixthPartDone = false;

        return Mode::HotfireIdle;
    }

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log(
            "ABORT during 3 Second Hotfire, returning to HotfireIdle");

        // Shut everything down
        sparkPlug.TurnOff();

        valveControl.CloseValve(ValveControl::ASIEthanol);
        valveControl.CloseValve(ValveControl::MainEthanol);
        valveControl.CloseValve(ValveControl::MainNitrous);
        valveControl.CloseValve(ValveControl::Purge);

        controller.tvc.Stop();

        sequenceStarted = false;

        return Mode::HotfireIdle;
    }

    return Mode::ThreeSecondHotfire;
}

Mode::Phase Mode::UpdateAbort(Controller &controller, ValveControl &valvecontrol, SparkPlug &sparkplug)
{
    Telemetry::GetInstance().Log("Closing all Valves and Sparkplug, EXITING");
    controller.tvc.Stop();
    CloseAllValvesAndSparkPlug(valvecontrol, sparkplug);
    exit(0);
}

bool Mode::Update(Navigation &navigation, Controller &controller, GPS &gps, Igniter &igniter, IMU &imu,
                  Magnetometer &magnetometer, ValveControl &valveControl, SparkPlug &sparkPlug,
                  PressureTransducer &pressureTransducer, LoadCell &loadCell, Camera &camera)
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

    // Get command and first see if it is to toggle any fusion sensors
    RF::Command command = RF::GetInstance().GetCommand();
    CheckForToggleSensorCommands(command, gps, camera, magnetometer);
    CheckForToggleValveCommands(command, valveControl, sparkPlug);
    /* Handle behavior based on current phase. Update phase*/
    switch (this->eCurrentMode)
    {
    case Standby:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateStandby(command, controller);
        break;
    case Calibration:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateCalibration(command, navigation, controller, currentTime);
        break;
    case ActuatorCalibration:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateActuatorCalibration(command, navigation, controller, currentTime);
        break;
    case TestTVC:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateTestTVC(command, navigation, controller, currentTime);
        break;
    case ChirpTVC:
        this->eCurrentMode = UpdateChirpTVC(navigation, controller, currentTime);
        break;
    case Idle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateIdle(command, navigation, controller, currentTime);
        break;
    case Launch:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateLaunch(command, navigation, controller, igniter, currentTime);
        break;
    case HotfireIdle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateHotfireIdle(command, navigation, valveControl, sparkPlug);
        break;
    case ASITest:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateASITest(command, navigation, valveControl, sparkPlug, currentTime);
        break;
    case WaterFlow:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateWaterFlow(command, navigation, valveControl, sparkPlug, currentTime);
        break;
    case ThreeSecondHotfire:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = Update3SecondHotfire(command, navigation, controller, valveControl, sparkPlug, currentTime);
        break;
    case Safe:
        this->eCurrentMode = UpdateSafeMode(navigation, controller, currentTime);
        break;
    case Abort:
        this->eCurrentMode = UpdateAbort(controller, valveControl, sparkPlug);
    case Terminate:
        return false;
    }

    return true;
}
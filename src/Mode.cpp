#include <chrono>
#include <cmath>
#include "Mode.hpp"
#include "Navigation.hpp"
#include "MissionConstants.hpp"
#include "Telemetry.hpp"
#include "RF.hpp"
#include "ValveControl.hpp"
#include "SparkPlug.hpp"
#include "PressureTransducer.hpp"
#include "LoadCell.hpp"
#include "GPS.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <iostream>
#include <thread>

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

    return Mode::Calibration;
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
    else if (seconds_since_start >= 10)
    {
        Telemetry::GetInstance().Log("Switching mode from test to idle");
        controller.Center();
        return Mode::Idle;
    }

    return Mode::TestTVC;
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
    else if (command == RF::Command::Ignite)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to launch");
        navigation.SetOnPad(false); // Disable pad updates during flight
        navigation.reset();
        return Mode::Launch;
    }
    else if (command == RF::Command::GoHotfireIdle)
    {
        Telemetry::GetInstance().Log("Switching mode from idle to HotfireIdle");
        return Mode::HotfireIdle;
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

    return Mode::Terminate;
}

// Liquid Propulsion State Implementations
Mode::Phase Mode::UpdateHotfireIdle(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug)
{
    navigation.UpdateNavigation();

    RF::Command command = RF::GetInstance().GetCommand();

    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT, EXITING");
        exit(0);
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
    else if (command == RF::Command::GoIdle)
    {
        Telemetry::GetInstance().Log("Switching mode from HotfireIdle to Idle");
        return Mode::Idle;
    }
    // Handle Valve commands
    else if (command == RF::Command::ValveNitrogenOpen)
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

    return Mode::HotfireIdle;
}

Mode::Phase Mode::UpdateASITest(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug,
                                double currentTime)
{
    static double startTime = currentTime;
    static bool sequenceStarted = false;
    double seconds_since_start = currentTime - startTime;

    if (!sequenceStarted)
    {
        Telemetry::GetInstance().Log("Starting ASI Test sequence");
        valveControl.OpenValve(ValveControl::ASIOxygen);
        sparkPlug.TurnOn();
        sequenceStarted = true;
    }

    navigation.UpdateNavigation();

    // Sequence timing (matching original hotfire.ino logic)
    if (seconds_since_start >= 0.3 && seconds_since_start < 2.3)
    {
        // Open ASI ethanol after 300ms
        if (seconds_since_start < 0.31)
        {
            valveControl.OpenValve(ValveControl::ASIEthanol);
        }
    }
    else if (seconds_since_start >= 2.3 && seconds_since_start < 2.6)
    {
        // Close ASI ethanol and turn off spark after 2 seconds
        if (seconds_since_start < 2.31)
        {
            valveControl.CloseValve(ValveControl::ASIEthanol);
            sparkPlug.TurnOff();
        }
    }
    else if (seconds_since_start >= 2.6)
    {
        // Close ASI oxygen after 2.3 seconds
        if (seconds_since_start < 2.61)
        {
            valveControl.CloseValve(ValveControl::ASIOxygen);
        }
        // Return to HotfireIdle after sequence completes
        if (seconds_since_start >= 3.0)
        {
            startTime = 0;
            sequenceStarted = false;
            return Mode::HotfireIdle;
        }
    }

    RF::Command command = RF::GetInstance().GetCommand();
    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT during ASI Test, EXITING");
        // Close all valves and turn off spark
        valveControl.CloseValve(ValveControl::ASIOxygen);
        valveControl.CloseValve(ValveControl::ASIEthanol);
        sparkPlug.TurnOff();
        exit(0);
    }

    return Mode::ASITest;
}

Mode::Phase Mode::UpdateWaterFlow(Navigation &navigation, ValveControl &valveControl, SparkPlug &sparkPlug,
                                  double currentTime)
{
    static double startTime = currentTime;
    static bool sequenceStarted = false;
    double seconds_since_start = currentTime - startTime;

    if (!sequenceStarted)
    {
        Telemetry::GetInstance().Log("Starting Water Flow sequence");
        valveControl.OpenValve(ValveControl::MainNitrous);
        sequenceStarted = true;
    }

    navigation.UpdateNavigation();

    // Sequence timing (matching original hotfire.ino logic)
    if (seconds_since_start >= 2.0 && seconds_since_start < 5.0)
    {
        // Open main ethanol after 2 seconds
        if (seconds_since_start < 2.01)
        {
            valveControl.OpenValve(ValveControl::MainEthanol);
        }
    }
    else if (seconds_since_start >= 5.0)
    {
        // Close both valves after 5 seconds total (3 seconds after ethanol opens)
        if (seconds_since_start < 5.01)
        {
            valveControl.CloseValve(ValveControl::MainNitrous);
            valveControl.CloseValve(ValveControl::MainEthanol);
        }
        // Return to HotfireIdle after sequence completes
        if (seconds_since_start >= 5.5)
        {
            Telemetry::GetInstance().Log("Water Flow sequence completed, returning to HotfireIdle");
            startTime = 0;
            sequenceStarted = false;
            return Mode::HotfireIdle;
        }
    }

    RF::Command command = RF::GetInstance().GetCommand();
    if (command == RF::Command::ABORT)
    {
        Telemetry::GetInstance().Log("ABORT during Water Flow, EXITING");
        // Close all valves
        valveControl.CloseValve(ValveControl::MainNitrous);
        valveControl.CloseValve(ValveControl::MainEthanol);
        exit(0);
    }

    return Mode::WaterFlow;
}

bool Mode::Update(Navigation &navigation, Controller &controller, GPS &gps, Igniter &igniter, IMU &imu, ValveControl &valveControl,
                  SparkPlug &sparkPlug, PressureTransducer &pressureTransducer, LoadCell &loadCell)
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
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateCalibration(navigation, controller, currentTime);
        break;
    case TestTVC:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateTestTVC(navigation, controller, currentTime);
        break;
    case Idle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateIdle(navigation, controller, imu, currentTime);
        break;
    case Launch:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, currentTime);
        break;
    case HotfireIdle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateHotfireIdle(navigation, valveControl, sparkPlug);
        break;
    case ASITest:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateASITest(navigation, valveControl, sparkPlug, currentTime);
        break;
    case WaterFlow:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, gps, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateWaterFlow(navigation, valveControl, sparkPlug, currentTime);
        break;
    case Safe:
        this->eCurrentMode = UpdateSafeMode(navigation, controller, currentTime);
        break;
    case Terminate:
        return false;
    }

    return true;
}

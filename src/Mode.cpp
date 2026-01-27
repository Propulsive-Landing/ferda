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
#include <iostream>
#include <sstream>
#include <string>
#include <iostream>
#include <thread>

Mode::Mode(Phase eInitialMode) : eCurrentMode(eInitialMode) {}

void Mode::UploadKmatrices()
{
    std::cout << "Enter 0,1, or 2 to choose Launching K_matrix" << std::endl;
    int launching_KMatrix;
    std::cin >> launching_KMatrix;

    std::cout << "Enter 0,1, or 2 to choose Landing K_matrix" << std::endl;
    int landing_KMatrix;
    std::cin >> landing_KMatrix;

    switch (launching_KMatrix)
    {
    case 0:
        LaunchKMatrix = "../Normal_Launch.csv";
        break;
    case 1:
        LaunchKMatrix = "../Lazy_Launch.csv";
        break;
    case 2:
        LaunchKMatrix = "../Aggressive_Launch.csv";
        break;
    }

    switch (landing_KMatrix)
    {
    case 0:
        LandKMatrix = "../Normal_Land.csv";
        break;
    case 1:
        LandKMatrix = "../Lazy_Land.csv";
        break;
    case 2:
        LandKMatrix = "../Aggressive_Land.csv";
        break;
    }
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
        controller.ImportControlParameters(LaunchKMatrix);
        controller.Center();
        return Mode::TestTVC;
    }
    else if (command == RF::Command::GoIdle)
    {
        Telemetry::GetInstance().Log("Switching mode from calibration to idle");
        UploadKmatrices();
        controller.ImportControlParameters(LaunchKMatrix);
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

void Mode::GetAccelBiasOffset(Navigation &navigation, Controller &controller, IMU &imu, double currentTime)
{
    static int loops = 1;
    std::tuple<double, double, double> accel = imu.GetBodyAcceleration();
    static double accel_x = 0;
    static double accel_y = 0;
    static double accel_z = 0;

    accel_x += std::get<0>(accel);
    accel_y += std::get<1>(accel);
    accel_z += std::get<2>(accel);

    if (loops == 100)
    {
        accel_x /= loops;
        accel_y /= loops;
        accel_z /= loops;

        imu.SetAccelBiasX(-accel_x);
        imu.SetAccelBiasY(-accel_y);
        imu.SetAccelBiasZ(-accel_z);
    }

    ++loops;
}

void Mode::GetGyroBiasOffset(Navigation &navigation, Controller &controller, IMU &imu, double currentTime)
{
    static int loops = 1;
    std::tuple<double, double, double> gyro = imu.GetBodyAngularRate();
    static double gyro_x = 0;
    static double gyro_y = 0;
    static double gyro_z = 0;

    gyro_x += std::get<0>(gyro);
    gyro_y += std::get<1>(gyro);
    gyro_z += std::get<2>(gyro);

    if (loops == 100)
    {
        gyro_x /= loops;
        gyro_y /= loops;
        gyro_z /= loops;

        imu.SetGyroBiasX(-gyro_x);
        imu.SetGyroBiasY(-gyro_y);
        imu.SetGyroBiasZ(-gyro_z);
    }

    ++loops;
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
        // GetAccelBiasOffset(navigation, controller, imu, currentTime);
        // GetGyroBiasOffset(navigation, controller, imu, currentTime);
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
    // Launch rocket and start Controller on first iteration
    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;
    static int startup = 1;

    if (startup == 1)
    {
        Telemetry::GetInstance().Log("Igniting LAUNCH MOTOR");
        igniter.Ignite(Igniter::IgnitionSpecifier::LAUNCH);
        controller.Start(seconds_since_start);
        startup = 0;
    }

    if (seconds_since_start > 0.050)
    {
        igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAUNCH);
    }

    navigation.UpdateNavigation();
    controller.UpdateLaunch(navigation, seconds_since_start);

    Eigen::Matrix<double, 12, 1> testState = navigation.GetNavigation();

    // If z acceleration is negative and the z height is not the starting height, then we should go to freefall
    if (testState(5) < -1 && testState(2) > 2)
    {
        Telemetry::GetInstance().Log("Switching mode from launch to freefall");
        controller.Center();
        return Mode::Freefall;
    }
    return Mode::Launch;
}

Mode::Phase Mode::UpdateFreefall(Navigation &navigation, Controller &controller, Igniter &igniter, double currentTime)
{

    // Continue to update navigation
    navigation.UpdateNavigation();

    // Get currentState
    Eigen::Matrix<double, 12, 1> currentState = navigation.GetNavigation();

    // If the current time is greater than the calibration time + motor thrust duration + and offset, then figure out the best time to ignite
    // TODO THIS LOGIC IS BAD, CURRENT TIME IS VARIABLE DEPENDING ON LAUNCH PROCEDURE
    double a = -9.81 / 2;
    double b = currentState(5) + (-9.81 * (MissionConstants::motor_thrust_duration * MissionConstants::motor_thrust_percentage));

    double average_landing_throttle = 1;

    double c = currentState(5) * (MissionConstants::motor_thrust_duration * MissionConstants::motor_thrust_percentage) + currentState(2) + -9.81 * 0.5 * pow((MissionConstants::motor_thrust_duration * MissionConstants::motor_thrust_percentage), 2) + average_landing_throttle * MissionConstants::second_motor_delta_x - MissionConstants::gse_height;

    double time_till_second_ignite = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);

    // Check to see if we should ignite
    if (time_till_second_ignite <= 0.0)
    {
        controller.ResetKIteration(currentTime);
        controller.UpdateLand(navigation, currentTime);
        Telemetry::GetInstance().Log("Igniting LAND MOTOR");
        igniter.Ignite(Igniter::IgnitionSpecifier::LAND);
        Telemetry::GetInstance().Log("Switching from Freefall to Land");
        return Mode::Land;
    }
    // Start the controller before second ignition
    else if (time_till_second_ignite <= MissionConstants::timeToStartControllerBeforeIgnite2)
    {
        controller.ImportControlParameters(LandKMatrix);
        controller.ResetKIteration(currentTime);
        controller.UpdateLand(navigation, currentTime);
    }

    return Mode::Freefall;
}

Mode::Phase Mode::UpdateLand(Navigation &navigation, Controller &controller, double currentTime, Igniter &igniter)
{
    // Create variables to hold time difference since being in this function
    static double startTime = currentTime;
    double seconds_since_start = currentTime - startTime;

    // Turn off ignitor after we have been in this function for 0.05 seconds
    if (seconds_since_start > 0.050)
    {
        igniter.DisableIgnite(Igniter::IgnitionSpecifier::LAND);
    }

    // Continue to update navigation and controller
    navigation.UpdateNavigation();
    controller.UpdateLand(navigation, currentTime);

    return Mode::Land;
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
        Telemetry::GetInstance().Log("Opening Nitrogen Valve");
        valveControl.OpenValve(ValveControl::Nitrogen);
    }
    else if (command == RF::Command::ValveNitrogenClose)
    {
        Telemetry::GetInstance().Log("Closing Nitrogen Valve");
        valveControl.CloseValve(ValveControl::Nitrogen);
    }
    else if (command == RF::Command::ValvePurgeOpen)
    {
        Telemetry::GetInstance().Log("Opening purge Valve");
        valveControl.OpenValve(ValveControl::Purge);
    }
    else if (command == RF::Command::ValvePurgeClose)
    {
        Telemetry::GetInstance().Log("Closing purge Valve");
        valveControl.CloseValve(ValveControl::Purge);
    }
    else if (command == RF::Command::ValveMainEthanolOpen)
    {
        Telemetry::GetInstance().Log("Opening Main Ethanol Valve");
        valveControl.OpenValve(ValveControl::MainEthanol);
    }
    else if (command == RF::Command::ValveMainEthanolClose)
    {
        Telemetry::GetInstance().Log("Closing Main Ethanol Valve");
        valveControl.CloseValve(ValveControl::MainEthanol);
    }
    else if (command == RF::Command::ValveMainNitrousOpen)
    {
        Telemetry::GetInstance().Log("Opening Main Nitrous Valve");
        valveControl.OpenValve(ValveControl::MainNitrous);
    }
    else if (command == RF::Command::ValveMainNitrousClose)
    {
        Telemetry::GetInstance().Log("Closing Main Nitrous Valve");
        valveControl.CloseValve(ValveControl::MainNitrous);
    }
    else if (command == RF::Command::ValveASIEthanolOpen)
    {
        Telemetry::GetInstance().Log("Opening ASI Ethanol Valve");
        valveControl.OpenValve(ValveControl::ASIEthanol);
    }
    else if (command == RF::Command::ValveASIEthanolClose)
    {
        Telemetry::GetInstance().Log("Closing Main Ethanol Valve");
        valveControl.CloseValve(ValveControl::ASIEthanol);
    }
    else if (command == RF::Command::ValveASIOxygenOpen)
    {
        Telemetry::GetInstance().Log("Opening ASI Oxygen Valve");
        valveControl.OpenValve(ValveControl::ASIOxygen);
    }
    else if (command == RF::Command::ValveASIOxygenClose)
    {
        Telemetry::GetInstance().Log("Closing ASI Oxygen Valve");
        valveControl.CloseValve(ValveControl::ASIOxygen);
    }
    else if (command == RF::Command::ValveNitrogenBleedOpen)
    {
        Telemetry::GetInstance().Log("Opening Nitrogen Bleed Valve");
        valveControl.OpenValve(ValveControl::NitrogenBleed);
    }
    else if (command == RF::Command::ValveNitrogenBleedClose)
    {
        Telemetry::GetInstance().Log("Closing Nitrogen Bleed Valve");
        valveControl.CloseValve(ValveControl::NitrogenBleed);
    }
    // Handle spark commands
    else if (command == RF::Command::SparkOn)
    {
        Telemetry::GetInstance().Log("Turning Spark on");
        sparkPlug.TurnOn();
    }
    else if (command == RF::Command::SparkOff)
    {
        Telemetry::GetInstance().Log("Turning Spark off");
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
            Telemetry::GetInstance().Log("ASI Test sequence completed, returning to HotfireIdle");
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

bool Mode::Update(Navigation &navigation, Controller &controller, Igniter &igniter, IMU &imu, ValveControl &valveControl,
                  SparkPlug &sparkPlug, PressureTransducer &pressureTransducer, LoadCell &loadCell)
{

    // Track total elapsed time and delta time
    static double currentTime = 0;
    static auto last_time = std::chrono::high_resolution_clock::now();
    // Helpful when running SIL
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    auto time_now = std::chrono::high_resolution_clock::now();
    unsigned int nanoseconds_since_start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - last_time).count();
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
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateCalibration(navigation, controller, currentTime);
        break;
    case TestTVC:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateTestTVC(navigation, controller, currentTime);
        break;
    case Idle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateIdle(navigation, controller, imu, currentTime);
        break;
    case Launch:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateLaunch(navigation, controller, igniter, currentTime);
        break;
    case Freefall:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateFreefall(navigation, controller, igniter, currentTime);
        break;
    case Land:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateLand(navigation, controller, currentTime, igniter);
        break;
    case Safe:
        this->eCurrentMode = UpdateSafeMode(navigation, controller, currentTime);
        break;
    case Terminate:
        return false;
    case HotfireIdle:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateHotfireIdle(navigation, valveControl, sparkPlug);
        break;
    case ASITest:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateASITest(navigation, valveControl, sparkPlug, currentTime);
        break;
    case WaterFlow:
        Telemetry::GetInstance().RunTelemetry(navigation, controller, pressureTransducer, loadCell, 0.05, 0.08);
        this->eCurrentMode = UpdateWaterFlow(navigation, valveControl, sparkPlug, currentTime);
        break;
    }

    return true;
}

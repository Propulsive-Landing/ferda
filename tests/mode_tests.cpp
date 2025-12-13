#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "Mode.hpp"
#include "RF.hpp"
#include "Controller.hpp"
#include "Navigation.hpp"
#include "IMU.hpp"
#include "Barometer.hpp"
#include "TVC.hpp"
#include "Igniter.hpp"

// Tiny Helper so failures are easier to read
static void PrintBanner(const std::string& Name) {
    std::cout<<"\n[TEST]" << Name <<"\n";
}

// Create the logs directories that Telmetry + RF
// (hardware test) expect.
// (They open ../logs/.. in their constructors)

static void EnsureLogDirsExist(){

    std::error_code ec;
    std::filesystem::create_directories("../logs", ec);
}

// Write commands to a file and redirect stdin to that file.
// RF::GetCommand() in your test hardware layer reads from stdin when available.
static void RedirectStdInToFile(const std::string &path, const std::string& contents) {
    {
        std::ofstream ofsCmds(path);
        ofsCmds << contents;
        ofsCmds.flush();
    }
    if (!std::freopen(path.c_str(), "r", stdin))
    {
        std::cerr << "Failed to redirect stdin to: " << path <<"\n";
        std::abort();
    }
}

int main()
{

    EnsureLogDirsExist();

    // Test 1: RF::ParseCommamd() 
    
    PrintBanner("RF::ParseCommand maps command strings correctly");
    {
        RF& rf = RF::GetInstance();
        
        assert(rf.ParseCommand("IncrementXTVC") == RF::Command::IncrementXTVC);
        assert(rf.ParseCommand("IncrementYTVC") == RF::Command::IncrementYTVC);
        assert(rf.ParseCommand("DecrementXTVC") == RF::Command::DecrementXTVC);
        assert(rf.ParseCommand("DecrementYTVC") == RF::Command::DecrementYTVC);

        // Trim behavior
        assert(rf.ParseCommand("   Ignite   \n") == RF::Command::Ignite);
        assert(rf.ParseCommand("\tABORT\r\n") == RF::Command::ABORT);

        // Unknown -> None
        assert(rf.ParseCommand("NotARealCommand") == RF::Command::None);
    }

    // Test 2: Mode::Update() "does it run properly" test for calibration
    // Avoids commands that invoke UploadKmatrices() (interactive cin)

    PrintBanner("Mode::Update runs and consumes stdin commands for calibration");
    {
        const std::string cmdFile = "mode_test_cmds.txt";

        RedirectStdInToFile(
            cmdFile,
            "IncrementXTVC\n"
            "IncrementYTVC\n"
            "DecrementXTVC\n"
            "DecrementYTVC\n"
        );
    
        TVC tvc;
        IMU imu;
        Barometer barometer;
        Navigation navigation(imu, barometer, tvc);
        Controller controller(tvc);
        Igniter igniter;

        Mode mode(Mode::Calibration);

        // Run several ticks — asserting only "no crash + returns true"
        for (int iTick = 0; iTick < 8; ++iTick)
        {
            const bool bOk = mode.Update(navigation, controller, igniter, imu);
            assert(bOk && "Mode::Update should return true while not terminated");
        }

        // After the file ends, RF::GetCommand() should become None; Update should still be fine.
        const bool bOkAfterEOF = mode.Update(navigation, controller, igniter, imu);
        assert(bOkAfterEOF);
        
    }
        // Test 3: Update without any redirected stdin 

    PrintBanner("Mode::Update runs safely with no scripted stdin");
    {
        TVC tvc;
        IMU imu;
        Barometer barometer;
        Navigation navigation(imu, barometer, tvc);
        Controller controller(tvc);
        Igniter igniter;

        Mode mode(Mode::Calibration);

        for (int iTick=0; iTick < 3; ++iTick)
        {
            const bool test = mode.Update(navigation, controller, igniter, imu);
            assert(test);
        }
    }    

    std::cout << "\nAll mode_tests.cpp asserts PASSED. \n";
    return 0;
};

#include <string>
#include <fstream>
#include <stdio.h>

#include "Mode.hpp"
#include "Navigation.hpp"

class Telemetry {
    private:
        Telemetry();
        ~Telemetry();


        void HardwareSaveFrame(Navigation& navigation, Controller& controller);

    public:
        std::ofstream Logs;
        std::ofstream HardwareSaved;

        void RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta, float RFSaveDelta);
        void Log(std::string message);

        static Telemetry& GetInstance()
        {
            static Telemetry telem;

            return telem;
        }

};

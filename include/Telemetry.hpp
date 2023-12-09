#include <string>
#include <fstream>
#include <stdio.h>

#include "Mode.hpp"
#include "Navigation.hpp"

class Telemetry {
    private:
        std::ofstream Logs;
        std::ofstream HardwareSaved;

        Telemetry();
        ~Telemetry();


        void HardwareSaveFrame(Navigation& navigation, Controller& controller);

    public:
        void RunTelemetry(Navigation& navigation, Controller& controller, float HardwareSaveDelta);
        void Log(std::string message);

        static Telemetry& GetInstance()
        {
            static Telemetry telem;

            return telem;
        }

};

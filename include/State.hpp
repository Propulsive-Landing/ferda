#pragma once

class State
{
    private:
        enum Mode
        {
            Idle,
            Launch,
            Land,
            Terminate 
        }

        Mode UpdateIdle();
        Mode UpdateLaunch();
        Mode UpdateLand();
    public:
        State() = default;
        bool Update(); 
};
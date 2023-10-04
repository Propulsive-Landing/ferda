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

        Mode eCurrentMode;

        Mode UpdateIdle();
        Mode UpdateLaunch();
        Mode UpdateLand();
    public:
        State(Mode eInitialMode);
        bool Update(); 
};
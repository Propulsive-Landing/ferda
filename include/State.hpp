#pragma once

class State
{
    public:
        enum Mode
            {
                Idle,
                Launch,
                Land,
                Terminate 
            };
        
        State(State::Mode eInitialMode);
        bool Update(); 
    private:
        State::Mode eCurrentMode;

        State::Mode UpdateIdle();
        State::Mode UpdateLaunch();
        State::Mode UpdateLand();

};
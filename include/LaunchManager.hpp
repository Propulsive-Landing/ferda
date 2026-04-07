#pragma once

#include "Navigation.hpp"
#include "Controller.hpp"
#include "Igniter.hpp"

class LaunchManager
{

public:
    enum class LaunchPhase { Takeoff, Ascend, Hover, Descend, Land };
    enum class ChangeAltitudePhase { Accelerate, ConstantVelocity, Decelerate };

    LaunchManager();

    // Step the launch manager. Returns true when the manager requests handing off to top-level Land mode.
    bool Step(Navigation &navigation, Controller &controller, Igniter &igniter, double currentTime);

    // configuration
    void SetHoverTargetAltitude(double alt) { hoverTargetAltitude = alt; }
    void SetHoverDuration(double t) { hoverDurationSeconds = t; }
    void SetVelocities(double slowVel, double fastVel) { slowReferenceVelocity = slowVel; fastReferenceVelocity = fastVel; }

    void Reset();

    LaunchPhase eLaunchPhase;
    ChangeAltitudePhase eAscendPhase;
    ChangeAltitudePhase eDescendPhase;

    double launchPhaseStartTime;
    double hoverDurationSeconds;
    double slowReferenceVelocity;
    double fastReferenceVelocity;
    double hoverTargetAltitude;
    double takeoffAltitudeThreshold;
    double descendTransitionAltitude;
    double accelerationMargin;

    double currentMaxAcceleration;
    double currentMaxDeceleration;
    double groundHeight;
    void SetGroundHeight(double h) { groundHeight = h; }
};

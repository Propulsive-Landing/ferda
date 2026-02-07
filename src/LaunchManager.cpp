#include "LaunchManager.hpp"
#include "Telemetry.hpp"
#include <cmath>

LaunchManager::LaunchManager()
{
    eLaunchPhase = LaunchPhase::Takeoff;
    eAscendPhase = ChangeAltitudePhase::Accelerate;
    eDescendPhase = ChangeAltitudePhase::Accelerate;
    launchPhaseStartTime = 0.0;
    hoverDurationSeconds = 15.0;
    slowReferenceVelocity = 0.5;
    fastReferenceVelocity = 5.0;
    hoverTargetAltitude = 50.0;
    takeoffAltitudeThreshold = 1.0;
    descendTransitionAltitude = 2.0;
    currentMaxAcceleration = 2.0;
    currentMaxDeceleration = 2.0;
    groundHeight = 0.2800;
}

void LaunchManager::SetCurrentMaxAcceleration(double a)
{
    currentMaxAcceleration = a;
}

void LaunchManager::SetCurrentMaxDeceleration(double d)
{
    currentMaxDeceleration = d;
}

void LaunchManager::Reset()
{
    eLaunchPhase = LaunchPhase::Takeoff;
    eAscendPhase = ChangeAltitudePhase::Accelerate;
    eDescendPhase = ChangeAltitudePhase::Accelerate;
}

bool LaunchManager::Step(Navigation &navigation, Controller &controller, Igniter &igniter, double currentTime)
{
    // keep navigation updated
    navigation.UpdateNavigation();
    Eigen::Matrix<double, 16, 1> testState = navigation.GetNavigation();
    double currentAltitude = testState(2);

    // Similar logic to previous Mode::UpdateLaunch but encapsulated here
    switch (eLaunchPhase)
    {
    case LaunchPhase::Takeoff:
        Telemetry::GetInstance().Log("Launch sub-mode: TAKEOFF (slow reference velocity)");
        controller.refVelocityZ = slowReferenceVelocity;
        if (currentAltitude > takeoffAltitudeThreshold)
        {
            eLaunchPhase = LaunchPhase::Ascend;
            launchPhaseStartTime = currentTime;
            eAscendPhase = ChangeAltitudePhase::Accelerate;
            Telemetry::GetInstance().Log("Transition: TAKEOFF -> ASCEND");
        }
        break;

    case LaunchPhase::Ascend:
        Telemetry::GetInstance().Log("Launch sub-mode: ASCEND");
        switch (eAscendPhase)
        {
        case ChangeAltitudePhase::Accelerate:
            Telemetry::GetInstance().Log("  Ascend: ACCELERATE phase");
            controller.refVelocityZ += currentMaxAcceleration * controller.loopTime;
            {
                double accelDecelerationDistance = (controller.refVelocityZ * controller.refVelocityZ) / (2.0 * currentMaxDeceleration);
                if (currentAltitude + accelDecelerationDistance >= hoverTargetAltitude)
                {
                    eAscendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Ascend: ACCELERATE -> DECELERATE (skipping ConstantVelocity)");
                    break;
                }
            }
            if (controller.refVelocityZ >= fastReferenceVelocity)
            {
                controller.refVelocityZ = fastReferenceVelocity;
                eAscendPhase = ChangeAltitudePhase::ConstantVelocity;
                Telemetry::GetInstance().Log("  Ascend: ACCELERATE -> CONSTANT_VELOCITY");
            }
            break;

        case ChangeAltitudePhase::ConstantVelocity:
            Telemetry::GetInstance().Log("  Ascend: CONSTANT_VELOCITY phase");
            controller.refVelocityZ = fastReferenceVelocity;
            {
                double decelerationDistance = (controller.refVelocityZ * controller.refVelocityZ) / (2.0 * currentMaxDeceleration);
                if (currentAltitude > hoverTargetAltitude - decelerationDistance)
                {
                    eAscendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Ascend: CONSTANT_VELOCITY -> DECELERATE");
                }
            }
            break;

        case ChangeAltitudePhase::Decelerate:
            Telemetry::GetInstance().Log("  Ascend: DECELERATE phase");
            controller.refVelocityZ -= currentMaxDeceleration * controller.loopTime;
            if (controller.refVelocityZ <= 0.0)
            {
                controller.refVelocityZ = 0.0;
                eLaunchPhase = LaunchPhase::Hover;
                launchPhaseStartTime = currentTime;
                Telemetry::GetInstance().Log("  Ascend: DECELERATE complete -> HOVER");
            }
            break;
        }

        if (currentAltitude >= hoverTargetAltitude && controller.refVelocityZ <= 0.1)
        {
            eLaunchPhase = LaunchPhase::Hover;
            launchPhaseStartTime = currentTime;
            controller.refVelocityZ = 0.0;
            Telemetry::GetInstance().Log("Transition: ASCEND -> HOVER (altitude reached)");
        }
        break;

    case LaunchPhase::Hover:
        Telemetry::GetInstance().Log("Launch sub-mode: HOVER (holding altitude)");
        controller.refVelocityZ = 0.0;
        if ((currentTime - launchPhaseStartTime) >= hoverDurationSeconds)
        {
            eLaunchPhase = LaunchPhase::Descend;
            launchPhaseStartTime = currentTime;
            eDescendPhase = ChangeAltitudePhase::Accelerate;
            Telemetry::GetInstance().Log("Transition: HOVER -> DESCEND");
        }
        break;

    case LaunchPhase::Descend:
        Telemetry::GetInstance().Log("Launch sub-mode: DESCEND");
        switch (eDescendPhase)
        {
        case ChangeAltitudePhase::Accelerate:
            Telemetry::GetInstance().Log("  Descend: ACCELERATE phase");
            controller.refVelocityZ -= currentMaxAcceleration * controller.loopTime;
            {
                double descendAccelDecelerationDistance = (std::abs(controller.refVelocityZ) * std::abs(controller.refVelocityZ)) / (2.0 * currentMaxDeceleration);
                if (currentAltitude - descendAccelDecelerationDistance <= descendTransitionAltitude)
                {
                    eDescendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Descend: ACCELERATE -> DECELERATE (skipping ConstantVelocity)");
                    break;
                }
            }
            if (controller.refVelocityZ <= -fastReferenceVelocity)
            {
                controller.refVelocityZ = -fastReferenceVelocity;
                eDescendPhase = ChangeAltitudePhase::ConstantVelocity;
                Telemetry::GetInstance().Log("  Descend: ACCELERATE -> CONSTANT_VELOCITY");
            }
            break;

        case ChangeAltitudePhase::ConstantVelocity:
            Telemetry::GetInstance().Log("  Descend: CONSTANT_VELOCITY phase");
            controller.refVelocityZ = -fastReferenceVelocity;
            {
                double descendDecelerationDistance = (fastReferenceVelocity * fastReferenceVelocity) / (2.0 * currentMaxDeceleration);
                if (currentAltitude < descendTransitionAltitude + descendDecelerationDistance)
                {
                    eDescendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Descend: CONSTANT_VELOCITY -> DECELERATE");
                }
            }
            break;

        case ChangeAltitudePhase::Decelerate:
            Telemetry::GetInstance().Log("  Descend: DECELERATE phase");
            controller.refVelocityZ += currentMaxDeceleration * controller.loopTime;
            if (controller.refVelocityZ >= -0.05)
            {
                controller.refVelocityZ = -slowReferenceVelocity;
                eLaunchPhase = LaunchPhase::LandSub;
                launchPhaseStartTime = currentTime;
                Telemetry::GetInstance().Log("  Descend: DECELERATE complete -> LAND_SUB");
            }
            break;
        }

        if (currentAltitude <= descendTransitionAltitude && std::abs(controller.refVelocityZ) <= 0.1)
        {
            eLaunchPhase = LaunchPhase::LandSub;
            launchPhaseStartTime = currentTime;
            controller.refVelocityZ = -slowReferenceVelocity;
            Telemetry::GetInstance().Log("Transition: DESCEND -> LAND_SUB (altitude reached)");
        }
        break;

    case LaunchPhase::LandSub:
        Telemetry::GetInstance().Log("Launch sub-mode: LAND (slow reference velocity, final approach)");
        controller.refVelocityZ = -slowReferenceVelocity;
        if (currentAltitude <= (groundHeight + 0.05) || std::abs(testState(5)) < 0.5)
        {
            Telemetry::GetInstance().Log("Hand off to top-level Land");
            controller.Center();
            return true; // request top-level transition to Land
        }
        break;
    }

    // integrate reference position
    controller.refPositionZ += controller.refVelocityZ * controller.loopTime;

    // Call controller update
    controller.UpdateLaunch(navigation, 0.0);

    return false;
}

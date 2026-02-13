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
    double currentVelocityZ = testState(5);

    // Similar logic to previous Mode::UpdateLaunch but encapsulated here
    switch (eLaunchPhase)
    {
    case LaunchPhase::Takeoff:
        controller.refVelocityZ = slowReferenceVelocity;
        if (controller.refPositionZ > takeoffAltitudeThreshold)
        {
            eLaunchPhase = LaunchPhase::Ascend;
            launchPhaseStartTime = currentTime;
            eAscendPhase = ChangeAltitudePhase::Accelerate;
            Telemetry::GetInstance().Log("Transition: TAKEOFF -> ASCEND");
        }
        break;

    case LaunchPhase::Ascend:
        switch (eAscendPhase)
        {
        case ChangeAltitudePhase::Accelerate:
            controller.refVelocityZ += currentMaxAcceleration * controller.loopTime;
            {
                double accelDecelerationDistance = (controller.refVelocityZ * controller.refVelocityZ) / (2.0 * currentMaxDeceleration);
                if (controller.refPositionZ + accelDecelerationDistance >= hoverTargetAltitude)
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
            controller.refVelocityZ = fastReferenceVelocity;
            {
                double decelerationDistance = (controller.refVelocityZ * controller.refVelocityZ) / (2.0 * currentMaxDeceleration);
                if (controller.refPositionZ > hoverTargetAltitude - decelerationDistance)
                {
                    eAscendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Ascend: CONSTANT_VELOCITY -> DECELERATE");
                }
            }
            break;

        case ChangeAltitudePhase::Decelerate:
            controller.refVelocityZ -= currentMaxDeceleration * controller.loopTime;
            if (controller.refVelocityZ <= 0.0)
            {
                controller.refVelocityZ = 0.0;
                controller.refPositionZ = hoverTargetAltitude;
                eLaunchPhase = LaunchPhase::Hover;
                launchPhaseStartTime = currentTime;
                Telemetry::GetInstance().Log("  Ascend: DECELERATE complete -> HOVER");
            }
            break;
        }

        if (controller.refPositionZ >= hoverTargetAltitude)
        {
            eLaunchPhase = LaunchPhase::Hover;
            launchPhaseStartTime = currentTime;
            controller.refVelocityZ = 0.0;
            controller.refPositionZ = hoverTargetAltitude;
            Telemetry::GetInstance().Log("Transition: ASCEND -> HOVER (altitude reached)");
        }
        break;

    case LaunchPhase::Hover:
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
        switch (eDescendPhase)
        {
        case ChangeAltitudePhase::Accelerate:
            controller.refVelocityZ -= currentMaxAcceleration * controller.loopTime;
            {
                double descendAccelDecelerationDistance = (std::abs(controller.refVelocityZ) * std::abs(controller.refVelocityZ)) / (2.0 * currentMaxDeceleration);
                if (controller.refPositionZ - descendAccelDecelerationDistance <= descendTransitionAltitude)
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
            controller.refVelocityZ = -fastReferenceVelocity;
            {
                double descendDecelerationDistance = (fastReferenceVelocity * fastReferenceVelocity) / (2.0 * currentMaxDeceleration);
                if (controller.refPositionZ < descendTransitionAltitude + descendDecelerationDistance)
                {
                    eDescendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Descend: CONSTANT_VELOCITY -> DECELERATE");
                }
            }
            break;

        case ChangeAltitudePhase::Decelerate:
            controller.refVelocityZ += currentMaxDeceleration * controller.loopTime;
            if (controller.refVelocityZ >= -slowReferenceVelocity)
            {
                controller.refVelocityZ = -slowReferenceVelocity;
                eLaunchPhase = LaunchPhase::Land;
                launchPhaseStartTime = currentTime;
                Telemetry::GetInstance().Log("  Descend: DECELERATE complete -> LAND");
            }
            break;
        }

        if (controller.refPositionZ <= descendTransitionAltitude)
        {
            eLaunchPhase = LaunchPhase::Land;
            launchPhaseStartTime = currentTime;
            controller.refVelocityZ = -slowReferenceVelocity;
            Telemetry::GetInstance().Log("Transition: DESCEND -> LAND (altitude reached)");
        }
        break;

    case LaunchPhase::Land:
        controller.refVelocityZ = -slowReferenceVelocity;
        
        // TODO: Revisit landing condition
        if (currentAltitude <= groundHeight + 0.1 && currentVelocityZ >= -0.1)
        {
            Telemetry::GetInstance().Log("LAND complete (ground contact). Transitioning to Safe mode.");
            controller.Center();
            return true; // request top-level transition to Safe mode
        }
        break;
    }

    // integrate reference position
    controller.refPositionZ += controller.refVelocityZ * controller.loopTime;

    // Call controller update
    controller.UpdateLaunch(navigation, 0.0);

    return false;
}

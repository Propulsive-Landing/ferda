#include "LaunchManager.hpp"
#include "Telemetry.hpp"
#include "MissionConstants.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

LaunchManager::LaunchManager()
{
    eLaunchPhase = LaunchPhase::Takeoff;
    eAscendPhase = ChangeAltitudePhase::Accelerate;
    eDescendPhase = ChangeAltitudePhase::Accelerate;
    launchPhaseStartTime = 0.0;
    hoverDurationSeconds = MissionConstants::kGuidanceHoverDurationSeconds;
    slowReferenceVelocity = MissionConstants::kGuidanceSlowReferenceVelocityMps;
    fastReferenceVelocity = MissionConstants::kGuidanceFastReferenceVelocityMps;
    hoverTargetAltitude = MissionConstants::kGuidanceHoverTargetAltitudeM;
    takeoffAltitudeThreshold = MissionConstants::kGuidanceTakeoffAltitudeThresholdM;
    descendTransitionAltitude = MissionConstants::kGuidanceDescendTransitionAltitudeM;
    currentMaxAcceleration = 1.0; // Placeholder that prevents division by zero until first update
    currentMaxDeceleration = 1.0;
    accelerationMargin = MissionConstants::kGuidanceAccelerationMargin;
}

void LaunchManager::Reset()
{
    eLaunchPhase = LaunchPhase::Takeoff;
    eAscendPhase = ChangeAltitudePhase::Accelerate;
    eDescendPhase = ChangeAltitudePhase::Accelerate;
}

bool LaunchManager::Step(RF::Command command, Navigation &navigation, Controller &controller, Igniter &igniter, double currentTime)
{
    // keep navigation updated
    navigation.UpdateNavigation();

    if (command == RF::Command::ABORT_PAD && eLaunchPhase != LaunchPhase::Descend)
    {
        Telemetry::GetInstance().Log("ABORT_PAD received, transitioning immediately to DESCEND");
        eLaunchPhase = LaunchPhase::Descend;
        eDescendPhase = ChangeAltitudePhase::Accelerate;
        launchPhaseStartTime = currentTime;
    }

    if (command == RF::Command::ABORT_GROUND && eLaunchPhase != LaunchPhase::Descend)
    {
        Telemetry::GetInstance().Log("ABORT_GROUND received, transitioning immediately to DESCEND and zeroing translational setpoint angles");
        controller.ZeroTranslationalSetpointAngles();
        eLaunchPhase = LaunchPhase::Descend;
        eDescendPhase = ChangeAltitudePhase::Accelerate;
        launchPhaseStartTime = currentTime;
    }

    Eigen::Matrix<double, 16, 1> testState = navigation.GetNavigation();
    double currentAltitude = testState(2);
    double currentVelocityZ = testState(5);

    const double currentMassKg = std::max(navigation.GetEstimatedMassKg(), 1e-3);
    const double maxUpwardNetAcceleration = std::max((MissionConstants::kEngineMaxThrust / currentMassKg) - MissionConstants::kGravity, 0.0);
    const double maxDownwardNetAcceleration = std::max(MissionConstants::kGravity - (MissionConstants::kEngineMinThrust / currentMassKg), 0.0);
    currentMaxAcceleration = accelerationMargin * maxUpwardNetAcceleration;
    currentMaxDeceleration = accelerationMargin * maxDownwardNetAcceleration;

    // Similar logic to previous Mode::UpdateLaunch but encapsulated here
    switch (eLaunchPhase)
    {
    case LaunchPhase::Takeoff:
        controller.refAccelerationZ = 0.0;
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
        switch (eAscendPhase)
        {
        case ChangeAltitudePhase::Accelerate:
            controller.refAccelerationZ = currentMaxAcceleration;
            controller.refVelocityZ += currentMaxAcceleration * controller.loopTime;
            {
                double accelDecelerationDistance = (currentMaxDeceleration > 1e-6)
                    ? (currentVelocityZ * currentVelocityZ) / (2.0 * currentMaxDeceleration)
                    : std::numeric_limits<double>::infinity();                
                    if (currentAltitude + accelDecelerationDistance >= hoverTargetAltitude)
                {
                    eAscendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Ascend: ACCELERATE -> DECELERATE (skipping ConstantVelocity)");
                    break;
                }
            }
            if (currentVelocityZ >= fastReferenceVelocity)
            {
                controller.refVelocityZ = fastReferenceVelocity;
                eAscendPhase = ChangeAltitudePhase::ConstantVelocity;
                Telemetry::GetInstance().Log("  Ascend: ACCELERATE -> CONSTANT_VELOCITY");
            }
            break;

        case ChangeAltitudePhase::ConstantVelocity:
            controller.refAccelerationZ = 0.0;
            controller.refVelocityZ = fastReferenceVelocity;
            {
                double decelerationDistance = (currentMaxDeceleration > 1e-6)
                    ? (currentVelocityZ * currentVelocityZ) / (2.0 * currentMaxDeceleration)
                    : std::numeric_limits<double>::infinity();
                if (currentAltitude > hoverTargetAltitude - decelerationDistance)
                {
                    eAscendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Ascend: CONSTANT_VELOCITY -> DECELERATE");
                }
            }
            break;

        case ChangeAltitudePhase::Decelerate:
            controller.refAccelerationZ = -currentMaxDeceleration;
            controller.refVelocityZ -= currentMaxDeceleration * controller.loopTime;
            if (currentVelocityZ <= 0.0)
            {
                controller.refVelocityZ = 0.0;
                controller.refPositionZ = hoverTargetAltitude;
                eLaunchPhase = LaunchPhase::Hover;
                launchPhaseStartTime = currentTime;
                Telemetry::GetInstance().Log("  Ascend: DECELERATE complete -> HOVER");
            }
            break;
        }

        if (currentAltitude >= hoverTargetAltitude)
        {
            eLaunchPhase = LaunchPhase::Hover;
            launchPhaseStartTime = currentTime;
            controller.refVelocityZ = 0.0;
            controller.refPositionZ = hoverTargetAltitude;
            Telemetry::GetInstance().Log("Transition: ASCEND -> HOVER (altitude reached)");
        }
        break;

    case LaunchPhase::Hover:
        controller.refAccelerationZ = 0.0;
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
            controller.refAccelerationZ = -currentMaxAcceleration;
            controller.refVelocityZ -= currentMaxAcceleration * controller.loopTime;
            {
                double descendAccelDecelerationDistance = (currentMaxDeceleration > 1e-6)
                    ? (currentVelocityZ * currentVelocityZ) / (2.0 * currentMaxDeceleration)
                    : std::numeric_limits<double>::infinity();
                if (currentAltitude - descendAccelDecelerationDistance <= descendTransitionAltitude)
                {
                    eDescendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Descend: ACCELERATE -> DECELERATE (skipping ConstantVelocity)");
                    break;
                }
            }
            if (currentVelocityZ <= -fastReferenceVelocity)
            {
                controller.refVelocityZ = -fastReferenceVelocity;
                eDescendPhase = ChangeAltitudePhase::ConstantVelocity;
                Telemetry::GetInstance().Log("  Descend: ACCELERATE -> CONSTANT_VELOCITY");
            }
            break;

        case ChangeAltitudePhase::ConstantVelocity:
            controller.refAccelerationZ = 0.0;
            controller.refVelocityZ = -fastReferenceVelocity;
            {
                double descendDecelerationDistance = (currentMaxDeceleration > 1e-6)
                    ? (fastReferenceVelocity * fastReferenceVelocity) / (2.0 * currentMaxDeceleration)
                    : std::numeric_limits<double>::infinity();
                if (currentAltitude < descendTransitionAltitude + descendDecelerationDistance)
                {
                    eDescendPhase = ChangeAltitudePhase::Decelerate;
                    Telemetry::GetInstance().Log("  Descend: CONSTANT_VELOCITY -> DECELERATE");
                }
            }
            break;

        case ChangeAltitudePhase::Decelerate:
            controller.refAccelerationZ = currentMaxDeceleration;
            controller.refVelocityZ += currentMaxDeceleration * controller.loopTime;
            if (currentVelocityZ >= -slowReferenceVelocity)
            {
                controller.refVelocityZ = -slowReferenceVelocity;
                eLaunchPhase = LaunchPhase::Land;
                launchPhaseStartTime = currentTime;
                Telemetry::GetInstance().Log("  Descend: DECELERATE complete -> LAND");
            }
            break;
        }

        if (currentAltitude <= descendTransitionAltitude)
        {
            eLaunchPhase = LaunchPhase::Land;
            launchPhaseStartTime = currentTime;
            controller.refVelocityZ = -slowReferenceVelocity;
            Telemetry::GetInstance().Log("Transition: DESCEND -> LAND (altitude reached)");
        }
        break;

    case LaunchPhase::Land:
        controller.refAccelerationZ = 0.0;
        controller.refVelocityZ = -slowReferenceVelocity;
        
        // TODO: Revisit landing condition
        if (currentAltitude <= 0.1 && currentVelocityZ >= -0.1)
        {
            Telemetry::GetInstance().Log("LAND complete (ground contact). Transitioning to Safe mode.");
            controller.UpdateSafe();
            return true; // request top-level transition to Safe mode
        }
        break;
    }

    // integrate reference position
    controller.refPositionZ += controller.refVelocityZ * controller.loopTime;

    // Call controller update
    controller.UpdateLaunch(navigation, 0.0);
    navigation.UpdateMassFractionEstimate(controller.GetCurrentThrustCommand());

    return false;
}

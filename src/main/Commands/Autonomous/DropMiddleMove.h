// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include "Subsystems/DoubleArm/DoubleArm.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Commands/Common/SetWrist/SetWrist.h"
#include "Commands/Common/SetCone/SetCone.h"
#include "Commands/Common/SetintakeSpeed/SetIntakeSpeed.h"
#include "Commands/Common/SetArmCoordinate/SetArmCoordinate.h"

#include "Commands/Common/SetArmCoordinate/SetArmCoordinate.h"

static frc2::CommandPtr DropMiddleMove(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    std::vector<pathplanner::PathPlannerTrajectory> outLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("OutLoading", { pathplanner::PathConstraints(2_mps, 1.5_mps_sq) });
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap{};
    pathplanner::SwerveAutoBuilder autoBuilder(
        [m_swerveChassis = m_swerveChassis]() { return m_swerveChassis->getOdometry(); },
        [m_swerveChassis = m_swerveChassis](auto initPose) { m_swerveChassis->resetOdometry(initPose); },
        m_swerveChassis->getKinematics(),
        pathplanner::PIDConstants(5.0, 0.0, 0.0),
        pathplanner::PIDConstants(0.5, 0.0, 0.0),
        [m_swerveChassis = m_swerveChassis](auto speeds) { m_swerveChassis->setModuleStates(speeds); },
        eventMap,
        { m_swerveChassis },
        true
    );

    return frc2::cmd::Sequence(
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, { 0.76_m, 0.22_m }).ToPtr(), //Middle
        SetCone(m_intake, true).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),
        SetArmCoordinate(m_doubleArm, { 0.21_m, 0.05_m }).ToPtr(), //Closed
        autoBuilder.fullAuto(outLoadingTrajectory)
    );
}


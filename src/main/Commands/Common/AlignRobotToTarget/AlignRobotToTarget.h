// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AlignRobotToTarget
    : public frc2::CommandHelper<frc2::CommandBase, AlignRobotToTarget> {
public:
    AlignRobotToTarget( SwerveChassis* swerveChassis, VisionManager* visionManager, std::string position );
    void calculateAlignPose();

    void Initialize() override;

    void Execute() override;

    void End( bool interrupted ) override;

    bool IsFinished() override;

private:
    /* Subsystems */
    SwerveChassis* swerveChassis;
    VisionManager* visionManager;

    /* AlignRobot variables using pathplanner */
    std::string position;
    pathplanner::PPSwerveControllerCommand* alignCommand;
    pathplanner::PathPlannerTrajectory trajectory;
    pathplanner::PathConstraints constraints = { 1_mps, 1_mps_sq };
    std::unordered_map<std::string, frc::Pose2d> positionMap{
      {"Center", {1.5_m,0_m,{180_deg}}},
      {"Right", {1_m,1_m,{180_deg}}},
      {"Left", {1_m,-1_m,{180_deg}}}
    };
};

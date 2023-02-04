// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"

class VisionManager: public frc2::SubsystemBase {
public:
    VisionManager( SwerveChassis* chassis );
    void toggleOdometry();
    void updateOdometry();
    frc::Pose2d calculateAlignPose( frc::Pose2d desiredPose );
    void calculateTrajectory( std::string position );
    frc2::SequentialCommandGroup alignRobotToTarget();
    void Periodic() override;

private:
    // PhotonVision 
    photonlib::PhotonCamera cameraEstimator{ "IMX219" };
    photonlib::PhotonCamera camera{ "IMX219" };
    photonlib::PhotonPipelineResult cameraResult;
    frc::Transform3d cameraToRobot{ {0_m, 0_m, 0.50_m}, frc::Rotation3d{} };
    frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField( frc::AprilTagField::k2023ChargedUp ) };
    photonlib::PhotonPoseEstimator* poseEstimator;

    // Subsystems
    SwerveChassis* chassis;

    //Update odometry
    bool toggleVisionOdometry = true;

    // AlignRobotMaps
    pathplanner::PathPlannerTrajectory trajectory;
    std::unordered_map<std::string, frc::Pose2d> positionMap{
      {"Center", {1.5_m,0_m,{180_deg}}},
      {"Right", {1_m,1_m,{180_deg}}},
      {"Left", {1_m,-1_m,{180_deg}}}
    };
};

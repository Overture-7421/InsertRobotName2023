// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager( SwerveChassis* swerveChassis ): swerveChassis( swerveChassis ) {}

void VisionManager::setAllianceColor() {
    // Set alliance color for the poseEstimator
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        tagLayout.SetOrigin( frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide );
    } else {
        tagLayout.SetOrigin( frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide );
    }

    // Set pose estimator
    poseEstimator = new photonlib::PhotonPoseEstimator{
        tagLayout,
        photonlib::PoseStrategy::LOWEST_AMBIGUITY,
        std::move( cameraEstimator ),
        cameraToRobot
    };
}

void VisionManager::updateOdometry() {
    /* Calculate pose using AprilTags */
    std::optional<photonlib::EstimatedRobotPose>poseResult = poseEstimator->Update();;

    if (poseResult) {
        swerveChassis->addVisionMeasurement( poseResult.value().estimatedPose.ToPose2d(), poseResult.value().timestamp );
    }
}

void VisionManager::Periodic() {}

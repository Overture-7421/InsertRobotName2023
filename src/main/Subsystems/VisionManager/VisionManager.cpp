// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager(SwerveChassis* swerveChassis): swerveChassis(swerveChassis) {}

//Set alliance color for the poseEstimator
void VisionManager::setAllianceColor() {
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();

    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
    } else {
        tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
    }

    poseEstimatorSet = true;
    poseEstimator = new photonlib::PhotonPoseEstimator{
            tagLayout,
            photonlib::PoseStrategy::LOWEST_AMBIGUITY,
            std::move(photonlib::PhotonCamera{ "IMX219" }),
            cameraToRobot
    };
}

//Update odometry with vision
void VisionManager::updateOdometry() {
    std::optional<photonlib::EstimatedRobotPose> poseResult = update(swerveChassis->getOdometry());

    if (poseResult.has_value()) {
        photonlib::EstimatedRobotPose pose = poseResult.value();
        swerveChassis->addVisionMeasurement(pose.estimatedPose.ToPose2d(), pose.timestamp);
    }
}

//Get EstimatedRobotPose from PhotonVision
std::optional<photonlib::EstimatedRobotPose> VisionManager::update(frc::Pose2d estimatedPose) {
    // poseEstimator->SetReferencePose(frc::Pose3d(estimatedPose));
    return poseEstimator->Update();
}

//GetPhotonPipeLineResult from PhotonVision
std::optional<photonlib::PhotonPipelineResult> VisionManager::getCameraResult() {
    return camera.GetLatestResult();
}

//Check if poseEstimator is set
bool VisionManager::isPoseEstimatorSet() {
    return poseEstimatorSet;
}

void VisionManager::Periodic() {
    if (isPoseEstimatorSet()) {
        updateOdometry();
    }
}

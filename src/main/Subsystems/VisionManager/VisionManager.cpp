// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager( SwerveChassis* chassis ): chassis( chassis ) {
    // Determine alliance Color
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        tagLayout.SetOrigin( frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide );
    } else {
        tagLayout.SetOrigin( frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide );
    }

    // Set pose estimator
    poseEstimator = new photonlib::PhotonPoseEstimator{ tagLayout, photonlib::PoseStrategy::LOWEST_AMBIGUITY, std::move( cameraEstimator ), cameraToRobot };

}

void VisionManager::toggleOdometry() {
    toggleVisionOdometry = !toggleVisionOdometry;
}

void VisionManager::updateOdometry() {
    /* Calculate pose using AprilTags */
    std::optional<photonlib::EstimatedRobotPose>poseResult = poseEstimator->Update();

    if (poseResult) {
        chassis->addVisionMeasurement( poseResult.value().estimatedPose.ToPose2d(), poseResult.value().timestamp );
    }
}

/* Method for calculating the desiredPosition for path generation */
frc::Pose2d VisionManager::calculateAlignPose( frc::Pose2d desiredPose ) {
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    if (result.HasTargets()) {
        int tagID = result.GetBestTarget().GetFiducialId();
        frc::Pose2d targetpose = tagLayout.GetTagPose( tagID ).value().ToPose2d();
        frc::Pose2d objectivePose = targetpose.TransformBy( { desiredPose.Translation(), desiredPose.Rotation() } );

        frc::SmartDashboard::PutNumber( "Objective/objX", objectivePose.X().value() );
        frc::SmartDashboard::PutNumber( "Objective/objY", objectivePose.Y().value() );
        return objectivePose;
    }
    return chassis->getOdometry();
}

void VisionManager::calculateTrajectory( std::string position ) {
    // Create PathPoints
    frc::Pose2d chassisPose = chassis->getOdometry();
    frc::Pose2d visionPose = calculateAlignPose( positionMap[position] );

    if (visionPose == chassisPose) {
        trajectory = pathplanner::PathPlannerTrajectory();
        return;
    }

    std::vector<pathplanner::PathPoint> pathPoints = {
        { chassisPose.Translation(), chassisPose.Rotation(), chassisPose.Rotation() },
        { visionPose.Translation(), visionPose.Rotation(), visionPose.Rotation() }
    };

    // Create trajector with constraints
    pathplanner::PathConstraints constraints = { 1_mps, 1_mps_sq };
    trajectory = pathplanner::PathPlanner::generatePath( constraints, pathPoints );
}

frc2::SequentialCommandGroup VisionManager::alignRobotToTarget() {

    return frc2::SequentialCommandGroup(
        frc2::InstantCommand( [this]() {return this->calculateTrajectory( "Center" );} ),
        pathplanner::PPSwerveControllerCommand(
            trajectory,
            [this]() { return chassis->getOdometry(); },
            chassis->getKinematics(),
            { 5,0,0 },
            { 5,0,0 },
            { 1,0,0 },
            [this]( auto speeds ) { chassis->setModuleStates( speeds ); },
            { chassis },
            false
        )
    );
}

void VisionManager::Periodic() {
    if (toggleVisionOdometry) {
        updateOdometry();
    }

}

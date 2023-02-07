// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignRobotToTarget.h"

AlignRobotToTarget::AlignRobotToTarget( SwerveChassis* swerveChassis, VisionManager* visionManager, std::string position )
    : swerveChassis( swerveChassis ), visionManager( visionManager ), position( position ) {
    AddRequirements( { swerveChassis, visionManager } );
}

/* Method for calculating the desiredPosition for path generation */
void AlignRobotToTarget::calculateAlignPose() {
    photonlib::PhotonPipelineResult result = visionManager->camera.GetLatestResult();
    if (result.HasTargets()) {
        int tagID = result.GetBestTarget().GetFiducialId();
        frc::Pose2d targetpose = visionManager->tagLayout.GetTagPose( tagID ).value().ToPose2d();

        frc::Pose2d visionPose = targetpose.TransformBy( { positionMap[position].Translation(), positionMap[position].Rotation() } );
        frc::Pose2d chassisPose = swerveChassis->getOdometry();

        // Delete when functional ************************
        frc::SmartDashboard::PutNumber( "Objective/objX", visionPose.X().value() );
        frc::SmartDashboard::PutNumber( "Objective/objY", visionPose.Y().value() );

        // Create PathPoints
        std::vector<pathplanner::PathPoint> pathPoints = {
            { chassisPose.Translation(), chassisPose.Rotation(), chassisPose.Rotation() },
            { visionPose.Translation(), visionPose.Rotation(), visionPose.Rotation() }
        };

        // Create trajector with constraints
        trajectory = pathplanner::PathPlanner::generatePath( constraints, pathPoints );
    }
}

// Called when the command is initially scheduled.
void AlignRobotToTarget::Initialize() {
    calculateAlignPose();

    alignCommand = new pathplanner::PPSwerveControllerCommand(
        trajectory,
        [this]() { return swerveChassis->getOdometry(); },
        swerveChassis->getKinematics(),
        { 5,0,0 },
        { 5,0,0 },
        { 1,0,0 },
        [this]( auto speeds ) { swerveChassis->setModuleStates( speeds ); },
        { swerveChassis },
        false
    );

    alignCommand->Schedule();
}

// Called repeatedly when this Command is scheduled to run
void AlignRobotToTarget::Execute() {}

// Called once the command ends or is interrupted.
void AlignRobotToTarget::End( bool interrupted ) {
    alignCommand->Cancel();
}

// Returns true when the command should end.
bool AlignRobotToTarget::IsFinished() {
    return false;
}

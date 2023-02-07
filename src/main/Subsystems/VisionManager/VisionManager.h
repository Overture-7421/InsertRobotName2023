// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/DriverStation.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"

class VisionManager: public frc2::SubsystemBase {
public:
    VisionManager( SwerveChassis* chassis );
    void setAllianceColor();
    void updateOdometry();
    std::optional<photonlib::EstimatedRobotPose> getEstimatedGlobalPose( const frc::Pose3d& prevEstimatedRobotPose );
    void Periodic() override;

    /* PhotonVision */
    photonlib::PhotonCamera camera{ "IMX219" };
    frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField( frc::AprilTagField::k2023ChargedUp ) };

private:
    /* PhotonVision */
    photonlib::PhotonCamera cameraEstimator{ "IMX219" };
    photonlib::PhotonPipelineResult cameraResult;
    frc::Transform3d cameraToRobot{ {0_m, 0_m, 0.50_m}, frc::Rotation3d{} };
    photonlib::PhotonPoseEstimator* poseEstimator;

    /* Subsystems */
    SwerveChassis* swerveChassis;
};

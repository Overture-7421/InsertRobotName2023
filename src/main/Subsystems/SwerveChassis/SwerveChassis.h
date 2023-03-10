// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/time.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/Joystick.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <numbers>
#include <AHRS.h>

#include <Subsystems/SwerveModule/SwerveModule.h>

class SwerveChassis: public frc2::SubsystemBase {
public:
    SwerveChassis();
    void setTargetAngle(double targetAngle);
    void setSpeed(frc::ChassisSpeeds speeds);
    void setWheelVoltage(double voltage);
    frc::Pose2d getOdometry();
    void resetOdometry(frc::Pose2d initPose);
    double getHeadingRate();
    const frc::SwerveDriveKinematics<4>& getKinematics();
    void addVisionMeasurement(frc::Pose2d pose, units::second_t latency);
    void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
    wpi::array<frc::SwerveModuleState, 4> getModuleStates();
    wpi::array<frc::SwerveModulePosition, 4> getModulePosition();
    double getPitch();
    double getYaw();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void resetNavx(double angle = 0);

private:
    SwerveModule backRightModule{ 1, 2, 16, 26.91, "BackRightModule" };
    SwerveModule backLeftModule{ 3, 4, 17, -61.4, "BackLeftModule" };
    SwerveModule frontLeftModule{ 5, 6, 18, -72.3, "FrontLeftModule" };
    SwerveModule frontRightModule{ 7, 8, 19, -47.95, "FrontRightModule" };

    double wheelVoltage;
    double targetAngle;

    double linearX;
    double linearY;
    double angular;

    std::array<frc::Translation2d, 4> modulePos{
        frc::Translation2d(10.36_in, 10.36_in),   // front left
        frc::Translation2d(10.36_in, -10.36_in),  // front right
        frc::Translation2d(-10.36_in, -10.36_in), // back right
        frc::Translation2d(-10.36_in, 10.36_in)   // back left
    };

    AHRS navx{ frc::SPI::Port::kMXP };
    frc::SwerveDriveKinematics<4> kinematics{ modulePos };

    std::array<frc::SwerveModulePosition, 4> odometryPos{
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      frontRightModule.getPosition(),
    };

    frc::SwerveDrivePoseEstimator<4> odometry{
      kinematics,
      frc::Rotation2d{},
      odometryPos,
      frc::Pose2d{}
    };

    frc::Field2d field2d;
};

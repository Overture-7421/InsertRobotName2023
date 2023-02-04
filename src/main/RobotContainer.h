// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include <vector>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Commands/Teleop/Drive/Drive.h"

class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

    // Generates auto with pathplanner
    frc2::CommandPtr CreateAuto( std::string pathName );

private:
    void ConfigureBindings();

    //Controllers
    frc::XboxController controller{ 0 };
    frc2::Trigger resetNavx{ [this] {return controller.GetYButton();} };
    frc2::Trigger alignCenter{ [this] {return controller.GetAButton();} }; //Change Button for final robot
    frc2::Trigger alignRight{ [this] {return controller.GetRightBumper();} }; //Change Button for final robot
    frc2::Trigger alignLeft{ [this] {return controller.GetLeftBumper();} }; //Change Button for final robot

    // Subsystems
    SwerveChassis swerveChassis;
    VisionManager visionManager{ &swerveChassis };

    //Auto
    frc::SendableChooser<std::string> pathChooser;
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

};

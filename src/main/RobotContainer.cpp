// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

    //Set default commands
    swerveChassis.SetDefaultCommand( Drive( &swerveChassis, &controller ) );
    visionManager.SetDefaultCommand( frc2::InstantCommand( [this]() {this->visionManager.updateOdometry();} ) );

    //Set choosers for auto
    pathChooser.SetDefaultOption( "OutOfCommunity&Balance", "OutOfCommunity&Balance" );
    pathChooser.AddOption( "Test1", "Test1" );
    pathChooser.AddOption( "Test2", "Test2" );
    pathChooser.AddOption( "Test3", "Test3" );
    pathChooser.AddOption( "Test4", "Test4" );
    pathChooser.AddOption( "Test5", "Test5" );
    pathChooser.AddOption( "Test6", "Test6" );

    frc::SmartDashboard::PutData( "Auto Chooser", &pathChooser );

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    resetNavx.OnTrue( frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr() );
    alignCenter.WhileTrue( AlignRobotToTarget( &swerveChassis, &visionManager, "Center" ).ToPtr() );
    alignRight.WhileTrue( AlignRobotToTarget( &swerveChassis, &visionManager, "Right" ).ToPtr() );
    alignLeft.WhileTrue( AlignRobotToTarget( &swerveChassis, &visionManager, "Left" ).ToPtr() );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // Set alliance color for pose estimation and correct on the fly path generation
    visionManager.setAllianceColor();
    return CreateAuto( pathChooser.GetSelected() );
}

frc2::CommandPtr RobotContainer::CreateAuto( std::string pathName ) {
    std::vector<pathplanner::PathPlannerTrajectory> examplePath = pathplanner::PathPlanner::loadPathGroup( pathName, { pathplanner::PathConstraints( 4_mps, 4_mps_sq ),
    pathplanner::PathConstraints( 4_mps, 4_mps_sq ),
    pathplanner::PathConstraints( .5_mps, 2_mps_sq ), } );

    eventMap.emplace( "Stage1", std::make_shared<frc2::PrintCommand>( "Salió en la terminal??" ) );


    pathplanner::SwerveAutoBuilder autoBuilder(
        [this]() { return swerveChassis.getOdometry(); }, // Function to supply current robot pose
        [this]( auto initPose ) { swerveChassis.resetOdometry( initPose ); }, // Function used to reset odometry at the beginning of auto
        swerveChassis.getKinematics(),
        pathplanner::PIDConstants( 5.0, 0.0, 0.0 ), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        pathplanner::PIDConstants( 0.5, 0.0, 0.0 ), // PID constants to correct for rotation error (used to create the rotation controller)
        [this]( auto speeds ) { swerveChassis.setModuleStates( speeds ); }, // Output function that accepts field relative ChassisSpeeds
        eventMap, // Our event map
        { &swerveChassis }, // Drive requirements, usually just a single drive subsystem
        true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    );

    frc2::CommandPtr autoCommand = autoBuilder.fullAuto( examplePath );

    return autoCommand;
}
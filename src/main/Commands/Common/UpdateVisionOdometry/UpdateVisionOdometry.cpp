// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "UpdateVisionOdometry.h"

UpdateVisionOdometry::UpdateVisionOdometry( VisionManager* visionManager ): visionManager( visionManager ) {
    // Use addRequirements() here to declare subsystem dependencies.
    AddRequirements( visionManager );
}

// Called when the command is initially scheduled.
void UpdateVisionOdometry::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void UpdateVisionOdometry::Execute() {
    visionManager->updateOdometry();
}

// Called once the command ends or is interrupted.
void UpdateVisionOdometry::End( bool interrupted ) {}

// Returns true when the command should end.
bool UpdateVisionOdometry::IsFinished() {
    return false;
}

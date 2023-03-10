// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
    intakeMotor.SetNeutralMode(NeutralMode::Brake);
    intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
    intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);
    intakeMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 0, 1));

    conePiston.Set(frc::DoubleSolenoid::Value::kReverse);
    wristPiston.Set(frc::DoubleSolenoid::Value::kForward);
}

void Intake::setVoltage(double voltage) {
    intakeMotor.SetVoltage(units::volt_t(voltage));
}

void Intake::setConeControl() {
    conePiston.Toggle();
}

void Intake::setConeAuto(bool state) {
    if (state) {
        conePiston.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        conePiston.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

void Intake::setWristControl() {
    wristPiston.Toggle();
}

void Intake::setWristAuto(bool state) {
    if (state) {
        wristPiston.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        wristPiston.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

void Intake::Periodic() {}
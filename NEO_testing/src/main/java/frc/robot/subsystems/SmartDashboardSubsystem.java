// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  public void updateAllTelemetry() {
    encoderTelemetry();
    velocityTelemetry();
  }

  public void encoderTelemetry() {
    SmartDashboard.putNumber("Left Encoder", RobotContainer.neoBranchSubsystem.getLeftShooterMotorEncoder());
    SmartDashboard.putNumber("Right Encoder", RobotContainer.neoBranchSubsystem.getRightShooterMotorEncoder());
  }

  public void velocityTelemetry() {
    SmartDashboard.putNumber("Left Velocity", RobotContainer.neoBranchSubsystem.getLeftShooterMotorVelocity());
    SmartDashboard.putNumber("Right Velocity", RobotContainer.neoBranchSubsystem.getRightShooterMotorVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllTelemetry();
  }
}

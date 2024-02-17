// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXSubsystem extends SubsystemBase {
  private AHRS navX;
  /** Creates a new NavXSubsystem. */
  public NavXSubsystem() {
    navX = new AHRS(SPI.Port.kMXP);
  }

  public double getRoll() {
    return navX.getRoll();
  }

  public double getPitch() {
    return navX.getPitch();
  }

  public double getYaw() {
    return navX.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pitch: ", getPitch());
    SmartDashboard.putNumber("Roll: ", getRoll());
    SmartDashboard.putNumber("Yaw: ", getYaw());
  }
}

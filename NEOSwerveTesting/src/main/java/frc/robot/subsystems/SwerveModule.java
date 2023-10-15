// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private CANSparkMax driveMotor;
  private CANSparkMax angleMotor;
  private SparkMaxAbsoluteEncoder angleEncoder;
  private SparkMaxPIDController pid;

  public SwerveModule() {
    configureDriveMotor();
  }
  //3 for turn, 4 for drive
  
  public void configureDriveMotor() {
    driveMotor = new CANSparkMax(4, MotorType.kBrushless);
    angleMotor = new CANSparkMax(3, MotorType.kBrushless);
    
    driveMotor.restoreFactoryDefaults();

    angleEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    pid = angleMotor.getPIDController();

    pid.setFeedbackDevice(angleEncoder);

    angleEncoder.setPositionConversionFactor(6.283185307179586);
    angleEncoder.setVelocityConversionFactor(6.283185307179586);

    angleEncoder.setInverted(true);

    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(0.0);
    pid.setPositionPIDWrappingMaxInput(6.283185307179586);

    // Configure PID values

    pid.setP(0.4);
    pid.setI(0.0);
    pid.setD(0.8);
    pid.setFF(0.0);
    pid.setOutputRange(-1.0,
        1.0);

    angleMotor.setIdleMode(IdleMode.kBrake);
    //angleMotor.setSmartCurrentLimit(20);

    driveMotor.clearFaults();

    driveMotor.setInverted(true);


    driveMotor.setCANTimeout(0);

    driveMotor.enableVoltageCompensation(12.0);
    // driveMotor.setSmartCurrentLimit(20.0);
    // driveMotor.setOpenLoopRampRate(0.25);
    // driveMotor.setClosedLoopRampRate(0.25);

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    driveMotor.burnFlash();
    angleMotor.burnFlash();
  }

  public void startDriveMotor(){
    driveMotor.set(0.25);
  }

  public void stopDriveMotor(){
    driveMotor.set(0);
  }

  public void startAngleMotor(){
    angleMotor.set(0.15);
  }

  public void stopAngleMotor(){
    angleMotor.set(0);
  }

  public double getDriveMotorEncoder(){
    return driveMotor.getEncoder().getPosition();
  }

  public double getAngleMotorEncoder(){
    return angleEncoder.getPosition();
  }

  public void goToAngle() {
    angleMotor.getPIDController().setReference(0, ControlType.kPosition);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("DE: ", getDriveMotorEncoder());
    SmartDashboard.putNumber("AE: ", getAngleMotorEncoder());
  }
}

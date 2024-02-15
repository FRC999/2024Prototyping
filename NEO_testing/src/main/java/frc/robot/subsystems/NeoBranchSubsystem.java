// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@SuppressWarnings({ "removal" })

public class NeoBranchSubsystem extends SubsystemBase {

    // NEO motors connected to Spark Max
    private CANSparkMax shooterMotorLeft;
    private CANSparkMax shooterMotorRight;
    private CANSparkMax shooterMotorLeader;
  
    // Built-in NEO encoders
    // Will be used with Velocity PID
    private RelativeEncoder shooterEncoderLeft;
    private RelativeEncoder shooterEncoderRight;
  
    // Necessary for hardware PID with Spark Max
    private SparkPIDController shooterPIDControllerLeft;
    private SparkPIDController shooterPIDControllerRight;
    
  /** Creates a new NeoBranchSubsystem. */
  public NeoBranchSubsystem() {
    // Check if need to initialize shooter

    shooterMotorLeft = new CANSparkMax(20, MotorType.kBrushless);
    shooterMotorRight = new CANSparkMax(21, MotorType.kBrushless);

    shooterEncoderLeft = shooterMotorLeft.getEncoder();
    shooterEncoderRight = shooterMotorRight.getEncoder();

    shooterPIDControllerLeft = shooterMotorLeft.getPIDController();
    shooterPIDControllerRight = shooterMotorRight.getPIDController();

    shooterMotorLeft.restoreFactoryDefaults();
    shooterMotorRight.restoreFactoryDefaults();

    shooterMotorLeft.clearFaults();
    shooterMotorRight.clearFaults();

    shooterMotorLeft.setInverted(false);
    shooterMotorRight.setInverted(true);

    shooterEncoderLeft.setPositionConversionFactor(2*Math.PI);
    shooterEncoderLeft.setVelocityConversionFactor(2*Math.PI/60);
    shooterEncoderRight.setPositionConversionFactor(2*Math.PI);
    shooterEncoderRight.setVelocityConversionFactor(2*Math.PI/60);

    shooterMotorLeft.setCANTimeout(0);
    shooterMotorRight.setCANTimeout(0);

    shooterMotorLeft.enableVoltageCompensation(12.0);
    shooterMotorLeft.setSmartCurrentLimit(40);
    shooterMotorLeft.setOpenLoopRampRate(0.25);
    shooterMotorLeft.setClosedLoopRampRate(0.25);

    shooterMotorRight.enableVoltageCompensation(12.0);
    shooterMotorRight.setSmartCurrentLimit(40);
    shooterMotorRight.setOpenLoopRampRate(0.25);
    shooterMotorRight.setClosedLoopRampRate(0.25);

    // sets which shooterMotorLeft is the leader and follower; set follower inversion if needed
    
    shooterMotorRight.follow(shooterMotorLeft,true);
    
    shooterMotorLeader = shooterMotorLeft;
    

    // // PID Controller setup
    // shooterPIDControllerLeft.setPositionPIDWrappingEnabled(false);
    shooterPIDControllerLeft.setP(0.05);
    shooterPIDControllerLeft.setI(0.000);
    shooterPIDControllerLeft.setD(0);
    shooterPIDControllerLeft.setIZone(500);
    shooterPIDControllerLeft.setFF(0);
    // kMaxOutput = 1 ; range is -1, 1
    shooterPIDControllerLeft.setOutputRange(-0.2,0.2);


    //shooterPIDControllerRight.setPositionPIDWrappingEnabled(false);
    shooterPIDControllerRight.setP(0.05);
    shooterPIDControllerRight.setI(0.000);
    shooterPIDControllerRight.setD(0);
    shooterPIDControllerRight.setIZone(500);
    shooterPIDControllerRight.setFF(0);
    // kMaxOutput = 1 ; range is -1, 1
    shooterPIDControllerRight.setOutputRange(-0.2,0.2);

    // kMaxOutput = 1 ; range is -1, 1
    // shooterPIDControllerB.setOutputRange(-Constants.GPMConstants.ShooterPIDConstants.kMaxOutput,
    // Constants.GPMConstants.ShooterPIDConstants.kMaxOu
  }

  /**
   * Run shooter with velocity using PID
   * @param speed
   */
  public void runShooterWithVelocity(double speed) {
    shooterMotorLeader.getPIDController().setReference((speed), ControlType.kVelocity);
  }

  /**
   * Run shooter with NON-PID power -1..1
   * @param power
   */
  public void runShooterWithPower(double power) {
    shooterMotorLeader.set(power);
  }

  /**
   * Run shooter with PID power -1..1; converts power to voltage
   * @param power
   */
  public void runShooterWithPowerPID(double power) {
    runShooterWithVoltagePID(MathUtil.clamp (power * 12.0, -12.0, 12.0));
  }

  /**
   * Run shooter with PID voltage; clamp voltage to nominal range
   * @param voltage
   */
  public void runShooterWithVoltagePID(double voltage) {
    shooterMotorLeader.getPIDController().setReference(MathUtil.clamp (voltage, -12.0, 12.0), ControlType.kVoltage);
  }

  public void stopShooter() {
    //shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    shooterMotorLeader.set(0);
  }

  // ===============================
  // ===== Shooter Individual Testing
  // ===============================\

  public void runLeftMotorWithVelocity(double speed) {
    shooterMotorLeft.getPIDController().setReference((speed), ControlType.kVelocity);
  }

  public void runRightMotorWithVelocity(double speed) {
    shooterMotorRight.getPIDController().setReference((speed), ControlType.kVelocity);
  }
  /**
   * Run shooter with NON-PID power -1..1
   * @param power
   */
  public void runLeftMotorWithPower(double power) {
    shooterMotorLeft.set(power);
  }

  public void runRightMotorWithPower(double power) {
    shooterMotorRight.set(power);
  }


  /**
   * Run shooter with PID power -1..1; converts power to voltage
   * @param power
   */
  public void runLeftWithPowerPID(double power) {
    runLeftMotorWithVoltagePID(MathUtil.clamp (power * 12.0, -12.0, 12.0));
  }

  public void runRightWithPowerPID(double power) {
    runRightMotorWithVoltagePID(MathUtil.clamp (power * 12.0, -12.0, 12.0));
  }

  /**
   * Run shooter with PID voltage; clamp voltage to nominal range
   * @param voltage
   */
  public void runLeftMotorWithVoltagePID(double voltage) {
    shooterMotorLeft.getPIDController().setReference(MathUtil.clamp (voltage, -12.0, 12.0), ControlType.kVoltage);
  }

  public void runRightMotorWithVoltagePID(double voltage) {
    shooterMotorRight.getPIDController().setReference(MathUtil.clamp (voltage, -12.0, 12.0), ControlType.kVoltage);
  }

  public void stopMotorLeft() {
    //shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    shooterMotorLeft.set(0);
  }

  public void stopMotorRight() {
    //shooterMotorLeader.getPIDController().setReference((0), ControlType.kVelocity);
    shooterMotorRight.set(0);
  }

  // ===============================
  // ===== Shooter telemetry methods
  // ===============================

  public double getLeftShooterMotorVelocity() {
    return shooterEncoderLeft.getVelocity();
  }

  public double getRightShooterMotorVelocity() {
    return shooterEncoderRight.getVelocity();
  }

    public double getLeftShooterMotorEncoder() {
    return shooterEncoderLeft.getPosition();
  }

  public double getRightShooterMotorEncoder() {
    return shooterEncoderRight.getPosition();
  }

  public void setPositionPID(double position) {
    shooterMotorLeader.getPIDController().setReference(position, ControlType.kPosition);
        // armEncoderZero is encoder position at ZERO degrees
        // So, the expected encoder position is armEncoderZero plus
        // the degrees angle multiplied by ARM_ENCODER_CHANGE_PER_DEGREE\
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

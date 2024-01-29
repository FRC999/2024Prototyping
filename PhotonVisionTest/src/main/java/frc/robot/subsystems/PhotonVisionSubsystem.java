// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
  // page 208
  public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

  // See
  // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
  // page 197
  public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

  // See
  // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
  // pages 4 and 5
  public static final double kFarTgtXPos = Units.feetToMeters(54);
  public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75)
      - Units.inchesToMeters(48.0 / 2.0);
  public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("Razer_Kiyo");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;

  public PhotonVisionSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    SmartDashboard.putBoolean("Has Target ", result.hasTargets());

    if (result.hasTargets()) {
      var imageCaptureTime = result.getTimestampSeconds();
      var camToTargetTrans = result.getBestTarget().getBestCameraToTarget();
      System.out.println("CTT"+camToTargetTrans);
      SmartDashboard.putNumber("X", camToTargetTrans.getX());
      SmartDashboard.putNumber("Y", camToTargetTrans.getY());
      SmartDashboard.putNumber("Z", camToTargetTrans.getZ());
      SmartDashboard.putString("TS", result.getBestTarget().toString());
      // var camPose =
      // Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
    }

  }
}

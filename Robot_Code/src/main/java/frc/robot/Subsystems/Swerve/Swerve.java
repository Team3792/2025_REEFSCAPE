// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.MatchData;
import frc.robot.Subsystems.Vision.Vision;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  SwerveModule frontLeft = new SwerveModule(ModuleConstants.kFrontLeftConfig, "Front Left");
  SwerveModule frontRight = new SwerveModule(ModuleConstants.kFrontRightConfig, "Front Right");
  SwerveModule backLeft = new SwerveModule(ModuleConstants.kBackLeftConfig, "Back left");
  SwerveModule backRight = new SwerveModule(ModuleConstants.kBackRightConfig, "Back right");
  SwerveModule[] modules = { frontLeft, frontRight, backLeft, backRight };

  Pigeon2 pigeon = new Pigeon2(HardwareMap.kPigeon.id());

  Vision vision = new Vision();
  Field2d field = new Field2d();
  Field2d tag = new Field2d();

  SwerveDrivePoseEstimator fieldPoseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.kKinematics,
      getPigeonRotation2d(),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      },
      new Pose2d());

  SwerveDrivePoseEstimator tagPoseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.kKinematics,
      getPigeonRotation2d(),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      },
      new Pose2d());

  public Swerve() {
    configureAutoBuilder();
    resetHeading(180);

    SmartDashboard.putData("Field Pose", field);
    SmartDashboard.putData("Tag Pose", tag);
  }

  public void resetHeading() {
    pigeon.reset();
    fieldPoseEstimator.resetRotation(new Rotation2d());
  }

  public void resetHeading(double yawDegrees) {
    pigeon.setYaw(yawDegrees);
    fieldPoseEstimator.resetRotation(new Rotation2d());
  }

  private void configureAutoBuilder() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      config = new RobotConfig(0, 0, new ModuleConfig(0, 0, 0, new DCMotor(0, 0, 0, 0, 0, 4), 0, 4)); // TODO: make this
                                                                                                      // accurate
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getFieldPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> drive(speeds, false),
        new PPHolonomicDriveController(
            SwerveConstants.kTranslationPIDConstants,
            SwerveConstants.kRotationPIDConstants),
        config,
        MatchData::flipFieldToRed,
        this);

     addDashboardWidget();
      }

  @Override
  public void periodic() {
    updateFieldOdemetry();
    updateFieldVision();
    showRobotPose();
  }

  public void driveForwardVoltage(double voltage) {
    for (SwerveModule m : modules) {
      m.driveVoltage(voltage);
    }
  }

  public void stop() {
    for (SwerveModule m : modules) {
      m.stop();
    }
  }

  public Command driveForwardCommand(double voltage, double seconds) {
    return this.startEnd(() -> driveForwardVoltage(voltage), this::stop).withTimeout(seconds);
  }

  private void updateFieldVision() {
    var fieldPoseEstimate = vision.getFieldPoseEstimate();
    if (fieldPoseEstimate.isPresent()) {
      System.out.println("vision present");
      fieldPoseEstimator.addVisionMeasurement(fieldPoseEstimate.get().estimatedPose.toPose2d(),
          fieldPoseEstimate.get().timestampSeconds);
    }
  }

  private void updateFieldOdemetry() {
    fieldPoseEstimator.update(getPigeonRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

  }

  public void updateTagVision() {
    var poseFromTarget = vision.getTagToRobot();
    if (poseFromTarget.isPresent()) {
      Pose2d pose = poseFromTarget.get().pose();
      tagPoseEstimator.addVisionMeasurement(pose, poseFromTarget.get().timeStampSeconds());

      double xTranslationMeters = pose.getX();
      double yTranslationMeters = pose.getY();
      double thetaDegrees = pose.getRotation().getDegrees();

      // field.setRobotPose(poseFromTarget.get().plus(new Transform2d(new
      // Translation2d(5, 5), new Rotation2d(0))));

      SmartDashboard.putNumber("vision/x", xTranslationMeters);
      SmartDashboard.putNumber("vision/y", yTranslationMeters);
      SmartDashboard.putNumber("vision/theta", thetaDegrees);
      SmartDashboard.putBoolean("vision/low_camera", true);
    } else {
      SmartDashboard.putBoolean("vision/low_camera", false);
    }
  }

  private void addDashboardWidget() {
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurnPosition().getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurnPosition().getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurnPosition().getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurnPosition().getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getPigeonRotation2d().getRadians(), null);
      }
    });
  }

  public void updateTagOdemetry() {
    tagPoseEstimator.update(getPigeonRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  private Pose2d getFieldPose() {
    return fieldPoseEstimator.getEstimatedPosition();
  }

  public Pose2d getTagPose() {
    return tagPoseEstimator.getEstimatedPosition();
  }

  private void showRobotPose() {
    field.setRobotPose(getFieldPose());

    tag.setRobotPose(getTagPose());

  }

  public Rotation2d getPigeonRotation2d() {
    return pigeon.getRotation2d();
  }

  public void drive(ChassisSpeeds speeds, Rotation2d rotation) {
    speeds = ChassisSpeeds.discretize(speeds, 0.002);

    // Convert field centric to robot centric if fieldCentric is true
    // if (fieldCentric) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
    // }

    // Using kinematics, determine individual module states
    SwerveModuleState[] desiredStates = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    // Apply states to each module
    for (int i = 0; i < 4; i++) {
      modules[i].setState(desiredStates[i]);
    }
  }

  public void drive(ChassisSpeeds speeds, boolean fieldCentric) {
    speeds = ChassisSpeeds.discretize(speeds, 0.002);

    // Convert field centric to robot centric if fieldCentric is true
    if (fieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPigeonRotation2d());
    }

    // Using kinematics, determine individual module states
    SwerveModuleState[] desiredStates = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    // Apply states to each module
    for (int i = 0; i < 4; i++) {
      modules[i].setState(desiredStates[i]);
    }
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return SwerveConstants.kKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  private void resetPose(Pose2d newPose) {
    fieldPoseEstimator.resetPose(newPose);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(targetStates[i]);
    }
  }
}

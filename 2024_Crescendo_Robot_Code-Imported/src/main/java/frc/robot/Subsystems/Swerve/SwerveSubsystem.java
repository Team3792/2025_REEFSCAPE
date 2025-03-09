// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Subsystems.Vision;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveSubsystem extends SubsystemBase {
  SwerveModule swerveModule;
  SwerveModule frontLeft = new SwerveModule(0, 1, 2, 0.975, false, "Front Left");
  SwerveModule frontRight = new SwerveModule(2, 3, 3, 0.14, false, "Front Right");
  SwerveModule backLeft = new SwerveModule(4, 5, 5,  0.36, false, "Back Left");
  SwerveModule backRight = new SwerveModule(6, 7, 4, 0.246, false, "Back Right");
  SwerveModule[] modules = {
    frontLeft, frontRight, backLeft, backRight
  };
  private final Field2d field = new Field2d();
  Pigeon2 pigeon = new Pigeon2(9);
  public Vision vision = new Vision();

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.kKinematics, getRotation2d(), new SwerveModulePosition[]{
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
    }, 
    new Pose2d()); 
    
    //For now, assume starting at origin.

  public SwerveSubsystem() {
    SmartDashboard.putData(field);
  }

  public void zeroHeading(double zero){
    pigeon.setYaw(zero);
  }

  public void zeroHeading(){
    pigeon.reset();
  }

  public void resetEncoders(){
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
    //zeroHeading();
  }



  public Rotation2d getRotation2d(){
    return pigeon.getRotation2d();
  }

  @Override
  public void periodic() {
    Optional<Pose2d> poseFromTarget = vision.getTagToCamera();
    if(poseFromTarget.isPresent()){
      Pose2d pose = poseFromTarget.get();//.plus(new Transform2d(-5, -5, new Rotation2d()));
      poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
      double xTranslationMeters = pose.getX(); //In m
      double yTranslationMeters = pose.getY();
      double thetaDegrees = pose.getRotation().getDegrees();

      //field.setRobotPose(poseFromTarget.get().plus(new Transform2d(new Translation2d(5, 5), new Rotation2d(0))));


      SmartDashboard.putNumber("vision/x", xTranslationMeters);
      SmartDashboard.putNumber("vision/y", yTranslationMeters);
      SmartDashboard.putNumber("vision/theta", thetaDegrees);
      SmartDashboard.putBoolean("vision/low_camera", true);
    } else{
      SmartDashboard.putBoolean("vision/low_camera", false);
    }


    //Update odemetry of pose estimator
    poseEstimator.update(getRotation2d(),
      new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
      );
      showReefPose();
  }

  public Pose2d getReefPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void showReefPose(){
    field.setRobotPose(getReefPose());
  }
 


  public void addVisionMeasurement(Pose3d visionMeasurement){
    poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), Timer.getFPGATimestamp());
  }

  public void resetPose(Pose2d newPose){
    poseEstimator.resetPosition(
      getRotation2d(), 
      new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()},
      newPose
    );
  }

  //Return the heading that should be fed into telop driving, uses the offset present in pose estimator
  public Rotation2d getDriverOrientedHeading(){
    
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double angle = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
      if (alliance.isPresent()) {
        if (alliance.get() == DriverStation.Alliance.Red){
          angle += 180; 
        }
      }
    return new Rotation2d(angle);   
  }


  //Reset pose to current pose at a different heading (0)
  public void resetPoseHeading(){
    double angle = 0;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Red){
        angle = 180; //Reflect angle if on red alliance
      }
    }
    poseEstimator.resetPosition(
      getRotation2d(), 
      new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()},
      new Pose2d(getReefPose().getTranslation(), Rotation2d.fromDegrees(angle))
    );
  }

  public void stopAll(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getSpeeds() {
    return Swerve.kKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.002 );
    System.out.println("x: " + robotRelativeSpeeds.vxMetersPerSecond + ", y: " + robotRelativeSpeeds.vyMetersPerSecond + ", theta: " + robotRelativeSpeeds.omegaRadiansPerSecond);

    SwerveModuleState[] targetStates = Swerve.kKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void driveFieldRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds.discretize(robotRelativeSpeeds, 0.002), poseEstimator.getEstimatedPosition().getRotation());
    //System.out.println("x: " + robotRelativeSpeeds.vxMetersPerSecond + ", y: " + robotRelativeSpeeds.vyMetersPerSecond + ", theta: " + robotRelativeSpeeds.omegaRadiansPerSecond);

    SwerveModuleState[] moduleStates = Constants.Swerve.kKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleState(moduleStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.kMaxSpeedM);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i]);
    }
  }

  public void setModuleState(SwerveModuleState[] desiredStates){
   //TODO: normalize wheel speeds
    
    //System.out.println(desiredStates[0].angle.getDegrees());
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}

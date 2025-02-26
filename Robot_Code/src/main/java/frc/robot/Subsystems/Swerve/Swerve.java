// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.Subsystems.Vision.Vision;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  SwerveModule frontLeft = new SwerveModule(ModuleConstants.kFrontLeftConfig);
  SwerveModule frontRight = new SwerveModule(ModuleConstants.kFrontRightConfig);
  SwerveModule backLeft = new SwerveModule(ModuleConstants.kBackLeftConfig);
  SwerveModule backRight = new SwerveModule(ModuleConstants.kBackRightConfig);

  SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

  Pigeon2 pigeon = new Pigeon2(HardwareMap.kPigeon);
  Vision vision = new Vision();

  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    SwerveConstants.kKinematics, 
    getPigeonRotation2d(), 
    new SwerveModulePosition[]{
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    }, 
    new Pose2d()
  );

  public Swerve() {
    configureAutoBuilder();
  }

  private void configureAutoBuilder(){
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      config = new RobotConfig(0, 0, new ModuleConfig(0, 0, 0, new DCMotor(0, 0,  0, 0, 0, 4), 0, 4)); //TODO: make this accurate
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    updatePoseEstimator();
  }
  
  private void updatePoseEstimator(){
    // Optional<Pose2d> poseFromTarget = vision.getTagToCamera();
    // if(poseFromTarget.isPresent()){
    //   Pose2d pose = poseFromTarget.get();//.plus(new Transform2d(-5, -5, new Rotation2d()));
    //   poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());

    //   double xTranslationMeters = pose.getX(); 
    //   double yTranslationMeters = pose.getY();
    //   double thetaDegrees = pose.getRotation().getDegrees();

    //   //field.setRobotPose(poseFromTarget.get().plus(new Transform2d(new Translation2d(5, 5), new Rotation2d(0))));


    //   SmartDashboard.putNumber("vision/x", xTranslationMeters);
    //   SmartDashboard.putNumber("vision/y", yTranslationMeters);
    //   SmartDashboard.putNumber("vision/theta", thetaDegrees);
    //   SmartDashboard.putBoolean("vision/low_camera", true);
    // } else{
    //   SmartDashboard.putBoolean("vision/low_camera", false);
    // }





    //Update odemetry of pose estimator
    poseEstimator.update(getPigeonRotation2d(),
      new SwerveModulePosition[]{
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
      );
  }

  private Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.002 );

    SwerveModuleState[] targetStates = SwerveConstants.kKinematics.toSwerveModuleStates(targetSpeeds);
    //setStates(targetStates);
  }

  public void driveFieldRelative(ChassisSpeeds speeds){

  }

  public Rotation2d getPigeonRotation2d(){
    return pigeon.getRotation2d();
  }

  public void drive(ChassisSpeeds speeds, boolean fieldCentric){
    //Convert field centric to robot centric if fieldCentric is true
    if(fieldCentric){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPigeonRotation2d());
    }

    //Using kinematics, determine individual module states
    SwerveModuleState[] desiredStates = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    //Apply states to each module
    for(int i = 0; i < 4; i++){
      //modules[i].setState(desiredStates[i]);
    }
  }

  private ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.kKinematics.toChassisSpeeds(
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    );
  }

  private void resetPose(Pose2d newPose){
    poseEstimator.resetPose(newPose);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(targetStates[i]);
    }
  }
}

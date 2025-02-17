// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  SwerveModule frontLeft = new SwerveModule(ModuleConstants.kFrontLeftConfig);
  SwerveModule frontRight = new SwerveModule(ModuleConstants.kFrontRightConfig);
  SwerveModule backLeft = new SwerveModule(ModuleConstants.kBackLeftConfig);
  SwerveModule backRight = new SwerveModule(ModuleConstants.kBackRightConfig);

  SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

  Pigeon2 pigeon = new Pigeon2(HardwareMap.kPigeon);

  public Swerve() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.002 );

    SwerveModuleState[] targetStates = SwerveConstants.kKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void driveFieldRelative(ChassisSpeeds speeds){

  }

  public Rotation2d getHeading(){
    return pigeon.getRotation2d();
  }

  public void drive(ChassisSpeeds speeds, boolean fieldCentric){
    //Convert field centric to robot centric if fieldCentric is true
    if(fieldCentric){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
    }

    //Using kinematics, determine individual module states
    SwerveModuleState[] desiredStates = SwerveConstants.kKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    //Apply states to each module
    for(int i = 0; i < 4; i++){
      modules[i].setState(desiredStates[i]);
    }
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(targetStates[i]);
    }
  }
}

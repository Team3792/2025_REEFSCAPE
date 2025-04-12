// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Vision.FieldGeometry;
import java.util.ArrayList;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommand extends Command {
  /** Creates a new PrecisionAlignCommand. */
  Swerve swerve;
  Pose2d goalPoseField, goalPoseTag;
  Trajectory trajectory;
  AlignType alignType;
  Pose2d tagPose;
  ProfiledPIDController xController = SwerveConstants.kTranslationAlignConfid.getController();
  ProfiledPIDController yController = SwerveConstants.kTranslationAlignConfid.getController();
  ProfiledPIDController tController = SwerveConstants.kRotationAlignPIDConfig.getController();

  Timer timer = new Timer();

  HolonomicDriveController driveController = new HolonomicDriveController(
    SwerveConstants.kTranslationAlignPIDConfig.getController(),
    SwerveConstants.kTranslationAlignPIDConfig.getController(),
    SwerveConstants.kRotationAlignPIDConfig.getController());

  Field2d field = new Field2d();

  public AlignToTagCommand(Swerve swerve, AlignType alignType, Pose2d tagRelativePose) {
    this.swerve = swerve;
    goalPoseTag = tagRelativePose;


    driveController.setTolerance(SwerveConstants.kAutoAlignTolerance);

    xController.setTolerance(SwerveConstants.kAutoAlignTolerance.getX());
    yController.setTolerance(SwerveConstants.kAutoAlignTolerance.getY());
    tController.setTolerance(SwerveConstants.kAutoAlignTolerance.getRotation().getDegrees());

    tController.enableContinuousInput(-180, 180);
    this.alignType = alignType;
    SmartDashboard.putData("Goal Pose", field);
    addRequirements(swerve);
  }

  public static enum AlignType{
    Reef, CoralStation, Processor
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = swerve.getFieldPose();
    tagPose = FieldGeometry.getClosestTagPose(pose, alignType);
    Pose2d tagRelativeRobotPose = pose.relativeTo(tagPose);
    //goalPoseField = FieldGeometry.getTargetPose(pose, alignType, goalPoseTag);
    field.setRobotPose(tagPose);
    //SmartDashboard.putData(goalP)
    //Pose2d startingPose = swerve.getTagPose();
    ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.lastFieldRelativeCommand, tagPose.getRotation());
    //trajectory = TrajectoryGenerator.generateTrajectory(startingPose, List.of(goalPoseField.getTranslation()), goalPoseField, SwerveConstants.kAutoAlignTrajectoryConfig);
    // trajectory = TrajectoryGenerator.generateTrajectory(new ControlVector(
    //   new double[] {
    //     startingPose.getX(), 
    //     fieldVelocity.getX()
    //   }, 
    //   new double[] {
    //     startingPose.getY(), 
    //     fieldVelocity.getY()
    //   })
    //   , new ArrayList<Translation2d>(),
    //   new ControlVector(
    //   new double[] {
    //     goalPoseField.getX(), 
    //     0
    //   }, 
    //   new double[] {
    //     goalPoseField.getY(), 
    //     0
    //   }), SwerveConstants.kAutoAlignTrajectoryConfig);
   
      timer.reset();

      xController.reset(tagRelativeRobotPose.getX(), velocity.vxMetersPerSecond);
      yController.reset(tagRelativeRobotPose.getY(), velocity.vyMetersPerSecond);
      tController.reset(tagRelativeRobotPose.getRotation().getDegrees(), velocity.omegaRadiansPerSecond*180.0/Math.PI);//, fieldVelocity.getAngle().getDegrees());

      xController.setGoal(goalPoseTag.getX());
      yController.setGoal(goalPoseTag.getY());
      tController.setGoal(goalPoseTag.getRotation().getDegrees());
  }





  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pointExecute();
    //trajectoryExecute();
  }

  private void trajectoryExecute(){
    State stateSetpoint = trajectory.sample(timer.get());
    ChassisSpeeds fieldChassisSpeeds = driveController.calculate(swerve.getFieldPose(), stateSetpoint, goalPoseField.getRotation());
    ChassisSpeeds robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldChassisSpeeds, swerve.getFieldPose().getRotation());
    swerve.drive(robotChassisSpeeds, false);
  }

  private void pointExecute(){
    Pose2d pose = swerve.getFieldPose().relativeTo(tagPose);
    double xOut = xController.calculate(pose.getX());
    // if(Math.abs(pose.getY())>0.5){
    //   xOut = 0.5;
    // }
    ChassisSpeeds fieldChassisSpeeds = new ChassisSpeeds(
      xOut,
      yController.calculate(pose.getY()),
      tController.calculate(pose.getRotation().getDegrees())
    );
    ChassisSpeeds robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldChassisSpeeds, pose.getRotation());
    swerve.drive(robotChassisSpeeds, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      swerve.stop(); //Only stop if the command fully aligned
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && tController.atGoal();//false; //driveController.atReference();
  }
}

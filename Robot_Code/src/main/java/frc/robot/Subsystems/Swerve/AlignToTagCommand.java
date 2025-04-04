// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.FieldGeometry;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommand extends Command {
  /** Creates a new PrecisionAlignCommand. */
  Swerve swerve;
  Pose2d goalPoseField, goalPoseTag;
  Trajectory trajectory;
  Timer timer = new Timer();

  HolonomicDriveController driveController = new HolonomicDriveController(
    SwerveConstants.kTranslationAlignPIDConfig.getController(),
    SwerveConstants.kTranslationAlignPIDConfig.getController(),
    SwerveConstants.kRotationAlignPIDConfig.getController());

  public AlignToTagCommand(Swerve swerve, Pose2d tagRelativePose) {
    this.swerve = swerve;
    goalPoseTag = tagRelativePose;
    driveController.setTolerance(SwerveConstants.kAutoAlignTolerance);
    

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPoseField = FieldGeometry.getTargetPose(swerve.getFieldPose(), goalPoseTag);
    Pose2d startingPose = swerve.getTagPose();
    Translation2d fieldVelocity = swerve.getVelocity();

    trajectory = TrajectoryGenerator.generateTrajectory(new ControlVector(
      new double[] {
        startingPose.getX(), 
        fieldVelocity.getX()
      }, 
      new double[] {
        startingPose.getY(), 
        fieldVelocity.getY()
      })
      , null,
      new ControlVector(
      new double[] {
        goalPoseField.getX(), 
        0
      }, 
      new double[] {
        goalPoseField.getY(), 
        0
      }), SwerveConstants.kAutoAlignTrajectoryConfig);
   
      timer.reset();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State stateSetpoint = trajectory.sample(timer.get());
    ChassisSpeeds fieldChassisSpeeds = driveController.calculate(swerve.getFieldPose(), stateSetpoint, goalPoseField.getRotation());
    ChassisSpeeds robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldChassisSpeeds, swerve.getFieldPose().getRotation());
    swerve.drive(robotChassisSpeeds, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveController.atReference();
  }
}

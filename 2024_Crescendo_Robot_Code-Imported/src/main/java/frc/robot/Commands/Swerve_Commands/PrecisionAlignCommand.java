// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve_Commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrecisionAlignCommand extends Command {
  /** Creates a new PrecisionAlignCommand. */
  SwerveSubsystem swerveSubsystem;
  VisionSubsystem visionSubsystem;

  PIDController xController = new PIDController(3, 0.0, 0);
  PIDController yController = new PIDController(3, 0.0, 0);
  PIDController thetaController = new PIDController(0.21, 0.0, 0);

  public static enum AlignType{
    LeftAlign,
    RightAlign
  }
  public PrecisionAlignCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, AlignType alignType) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;

    

    xController.setSetpoint(0.05);
    xController.setTolerance(0.005);

    yController.setTolerance(0.005);


    thetaController.setSetpoint(180);
    thetaController.enableContinuousInput(-180.0, 180.0);
    thetaController.setTolerance(0.5);

    if(alignType == AlignType.LeftAlign){
      yController.setSetpoint(-0.18);
    } else {
      yController.setSetpoint(0.18);
    }


    addRequirements(swerveSubsystem, visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotReefPose = swerveSubsystem.getReefPose();
    double xTranslationMeters = robotReefPose.getX();
    double yTranslationMeters = robotReefPose.getY();
    double thetaDegrees = robotReefPose.getRotation().getDegrees();


      //System.out.println("x: " + xTranslationMeters + ", y: " + yTranslationMeters + ", theta: " + thetaDegrees);
      
      swerveSubsystem.driveFieldRelative(new ChassisSpeeds(
        clamp(-0.7, 0.7, xController.calculate(xTranslationMeters)),
        clamp(-0.5, 0.5, yController.calculate(yTranslationMeters)),
        clamp(-1, 1, thetaController.calculate(thetaDegrees))
      ));
    }
  

  public double clamp(double min, double max, double value){
    if(value > max){
      return max;
    } else if(value < min){
      return min;
    }
    return value;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}

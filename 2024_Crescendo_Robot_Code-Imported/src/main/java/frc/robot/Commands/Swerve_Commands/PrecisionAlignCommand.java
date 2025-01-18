// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve_Commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrecisionAlignCommand extends Command {
  /** Creates a new PrecisionAlignCommand. */
  SwerveSubsystem swerveSubsystem;
  Vision vision;

  PIDController xController = new PIDController(0.05, 0, 0);
  PIDController yController = new PIDController(0, 0, 0);
  PIDController thetaController = new PIDController(0.0, 0, 0);

  public static enum AlignType{
    LeftAlign,
    RightAlign
  }
  public PrecisionAlignCommand(SwerveSubsystem swerveSubsystem, Vision vision, AlignType alignType) {
    this.swerveSubsystem = swerveSubsystem;
    this.vision = vision;

    

    xController.setSetpoint(0.5);


    thetaController.setSetpoint(180);
    thetaController.enableContinuousInput(-180.0, 180.0);

    if(alignType == AlignType.LeftAlign){
      yController.setSetpoint(0.5);
    } else {
      yController.setSetpoint(-0.5);
    }


    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Transform3d> cameraToTarget = vision.getTagToCamera();
    if(cameraToTarget.isPresent()){
      double xTranslationMeters = cameraToTarget.get().getX(); //In m
      double yTranslationMeters = cameraToTarget.get().getY();
      double thetaDegrees = cameraToTarget.get().getRotation().getZ() * 180.0 / Math.PI; //In 


      SmartDashboard.putNumber("x", xTranslationMeters);
      SmartDashboard.putNumber("y", yTranslationMeters);
      SmartDashboard.putNumber("theta", thetaDegrees);


      //System.out.println("x: " + xTranslationMeters + ", y: " + yTranslationMeters + ", theta: " + thetaDegrees);

      swerveSubsystem.driveRobotRelative(new ChassisSpeeds(
        xController.calculate(xTranslationMeters),
        yController.calculate(yTranslationMeters),
        thetaController.calculate(-thetaDegrees)
      ));
    }

    // } else {
    //   System.out.println("No Camera Results");
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

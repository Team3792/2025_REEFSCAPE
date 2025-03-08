// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommand extends Command {
  /** Creates a new PrecisionAlignCommand. */
  Swerve swerve;

  PIDController xController = SwerveConstants.kTranslationAlignPIDConfig.getController();
  PIDController yController = SwerveConstants.kTranslationAlignPIDConfig.getController();
  PIDController thetaController = SwerveConstants.kRotationAlignPIDConfig.getController();

  public static enum AlignType{
    Center,
    LeftAlgae,
    RightAlgae
  }

  public AlignToTagCommand(Swerve swerve, Translation2d align) {
    this.swerve = swerve;

    

    xController.setSetpoint(align.getX());
    xController.setTolerance(SwerveConstants.kAutoAlignTranslationTolerance);

    yController.setSetpoint(align.getY());
    yController.setTolerance(SwerveConstants.kAutoAlignTranslationTolerance);


    thetaController.setSetpoint(180);
    thetaController.enableContinuousInput(-180.0, 180.0);
    thetaController.setTolerance(0.5);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.updateTagVision();
    Pose2d robotReefPose = new Pose2d(); //swerve.getReefPose();
    double xTranslationMeters = robotReefPose.getX();
    double yTranslationMeters = robotReefPose.getY();
    double thetaDegrees = robotReefPose.getRotation().getDegrees();


      //System.out.println("x: " + xTranslationMeters + ", y: " + yTranslationMeters + ", theta: " + thetaDegrees);
      
      swerve.drive(new ChassisSpeeds(
        clamp(-0.7, 0.7, xController.calculate(xTranslationMeters)),
        clamp(-0.5, 0.5, yController.calculate(yTranslationMeters)),
        clamp(-1, 1, thetaController.calculate(thetaDegrees))
      ), true);
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

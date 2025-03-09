// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve_Commands;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToAlgae extends Command {
  /** Creates a new AlignToAlgae. */
  SwerveSubsystem swerveSubsystem;
  PIDController thetaController = new PIDController(0.1, 0.0, 0);
  PIDController xController = new PIDController(200, 0, 0);
  PIDController yController = new PIDController(200, 0, 0);

  public AlignToAlgae(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    thetaController.enableContinuousInput(-180, 180);
    thetaController.setSetpoint(0);

    xController.setSetpoint(1/350.0);
    yController.setSetpoint(0);

    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> bestTarget = swerveSubsystem.vision.getBestTargetOptional();
    if(bestTarget.isPresent()){
      List<TargetCorner> targetCorners = bestTarget.get().getMinAreaRectCorners();
      double maxX, minX;
      maxX = minX = targetCorners.get(0).x;

      for(int i = 1; i < 4; i++){
        if(targetCorners.get(i).x < minX){
          minX = targetCorners.get(i).x;
        } else if(targetCorners.get(i).x > maxX){
          maxX = targetCorners.get(i).x;
        }
      }

      SmartDashboard.putNumber("vision/algae_width", maxX-minX);
      double distance = 1/(maxX-minX);
      double yaw = bestTarget.get().getYaw() * Math.PI/180.0;
      double x = distance * Math.cos(yaw);
      double y = distance * Math.sin(yaw);

      double xOutput = clamp(-1, 1, -xController.calculate(x));
      double yOutput = clamp(-1, 1, yController.calculate(y));
      double thetaOutput = clamp(-1, 1, thetaController.calculate(yaw * 180.0/Math.PI));
      //System.out.println(maxX-minX);



      swerveSubsystem.driveRobotRelative(new ChassisSpeeds(xOutput, yOutput, thetaOutput));
    }
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
    return false;
  }
}

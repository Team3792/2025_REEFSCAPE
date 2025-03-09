// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.LinearJoystickMap;
import frc.robot.Util.LinearJoystickMap.DriveMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualDriveCommand extends Command {
  /** Creates a new DriveCommand. */
  Swerve swerve;
  Supplier<Boolean> xySlowMode, tSlowMode;
  LinearJoystickMap xMap, yMap, tMap;

  public ManualDriveCommand(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> t, Supplier<Boolean> xySlowMode, Supplier<Boolean> tSlowMode) {
    this.swerve = swerve;

    xMap = new LinearJoystickMap(x, SwerveConstants.kSlowLinearVelocity, SwerveConstants.kMaxLinearVelocity, SwerveConstants.kDeadBandValue);
    yMap = new LinearJoystickMap(y, SwerveConstants.kSlowLinearVelocity, SwerveConstants.kMaxLinearVelocity, SwerveConstants.kDeadBandValue);
    tMap = new LinearJoystickMap(t, SwerveConstants.kSlowLinearVelocity, SwerveConstants.kMaxOmega, SwerveConstants.kDeadBandValue);

    this.xySlowMode = xySlowMode;
    this.tSlowMode = tSlowMode;
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
      new ChassisSpeeds(
        xMap.calculate(getDriveMode(xySlowMode.get())), 
        yMap.calculate(getDriveMode(xySlowMode.get())), 
        tMap.calculate(getDriveMode(tSlowMode.get()))
        ), 
        true);
  }

  private DriveMode getDriveMode(boolean slowMode){
    return slowMode ? DriveMode.Slow : DriveMode.Fast;
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

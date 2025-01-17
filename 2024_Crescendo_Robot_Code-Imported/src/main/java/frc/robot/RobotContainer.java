// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.Swerve_Commands.SwerveDriveCommand;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Util.RobotState;
import edu.wpi.first.wpilibj2.command.button.Trigger;




public class RobotContainer {
  //Create subsystems
  PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
 
  //Create joysticks and buttons
  CommandPS5Controller driverJoystick = new CommandPS5Controller(0);

  //Override buttons
  Trigger driverOverride = driverJoystick.touchpad();



  SwerveDriveCommand driveCommand = new SwerveDriveCommand(
      swerveSubsystem,
        () -> -driverJoystick.getLeftY(),
        () -> -driverJoystick.getLeftX(),
        () -> -driverJoystick.getRightX()
      );

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  

  public RobotContainer() {
    //Default Commands
    powerDistribution.clearStickyFaults();
    
    swerveSubsystem.setDefaultCommand(driveCommand);

    Shuffleboard.getTab("Play").addBoolean("Ready To Shoot", () -> RobotState.readyToShoot);
    Shuffleboard.getTab("Play").addBoolean("Ready To Intake", () -> RobotState.readyToIntake);
    Shuffleboard.getTab("Play").addBoolean("Has Note", () -> RobotState.hasNote);
    
  
    //SmartDashboard.putData("Auto Options", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    //Driving
    driverJoystick.povLeft().onTrue(new InstantCommand(() -> swerveSubsystem.resetEncoders()));
    driverJoystick.povRight().onTrue(new InstantCommand(() -> swerveSubsystem.resetPoseHeading()));

    
    //Heading Lock
    driverJoystick.triangle().onTrue(new InstantCommand(() -> driveCommand.addHeadingLock(0)));
    driverJoystick.square().onTrue(new InstantCommand(() -> driveCommand.addHeadingLock(90)));
    driverJoystick.cross().onTrue(new InstantCommand(() -> driveCommand.addHeadingLock(180)));
    driverJoystick.circle().onTrue(new InstantCommand(() -> driveCommand.addHeadingLock(-90)));



  
    
  }

  public Command getAutonomousCommand() {

    //shooterSubsystem.pitchZeroed = false;
    //return new TwoNoteAutoCenter(swerveSubsystem, intakeSubsystem, shooterSubsystem, feederSubsystem, ledSubsystem);

    //return Commands.none();
    return autoChooser.getSelected();
  }
}

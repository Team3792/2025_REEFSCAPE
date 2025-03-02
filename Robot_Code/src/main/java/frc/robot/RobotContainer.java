// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.server.Operation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntakeConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstants;
import frc.robot.Subsystems.Coral.Coral;
import frc.robot.Subsystems.Coral.CoralConstants;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.LED.LEDConstants;
import frc.robot.Subsystems.Swerve.DriveCommand;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {

  PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  // Create controllers
  CommandPS5Controller driverController = new CommandPS5Controller(0);
  CommandPS5Controller operatorController = new CommandPS5Controller(1);
  Joystick driveJoystick = new Joystick(2);
  Trigger r1 = new JoystickButton(driveJoystick, 1);
  Trigger r2 = new JoystickButton(driveJoystick, 2);

  // Create subsystems
  Climb climb = new Climb();
  AlgaeIntake algaeIntake = new AlgaeIntake();
   Coral coral = new Coral();
  // LED led = new LED();
  Swerve swerve = new Swerve();

 private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    pdh.clearStickyFaults();
    NamedCommands.registerCommand("IntakePosition", coral.holdAngleCommand(CoralConstants.kIntakePosition).withTimeout(CoralConstants.kAutoIntakeTime));
    NamedCommands.registerCommand("DumpPosition", coral.holdAngleCommand(CoralConstants.kDumpPosition).withTimeout(CoralConstants.kAutoDumpTime));
    swerve.setDefaultCommand(new DriveCommand(
      swerve, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -driverController.getRightX()));//() -> driverController.getL2Axis()-driverController.getR2Axis()));
    
    configureBindings(); 
    // swerve.setDefaultCommand(new DriveCommand(
    //   swerve, () -> -driveJoystick.getRawAxis(1), () -> -driveJoystick.getRawAxis(0), () -> -driveJoystick.getRawAxis(2)));//() -> driverController.getL2Axis()-driverController.getR2Axis()));
    

    // defaults coral manipulator to stop
    // coralSubsystem.setDefaultCommand(coralSubsystem.setVoltageCommandFactory(0,0));
    //led.setDefaultCommand(led.setLEDPatternCommand(LEDConstants.kIdleLED));

      // For convenience a programmer could change this when going to competition.
      boolean isCompetition = false;

      // Build an auto chooser. This will use Commands.none() as the default option.
      // As an example, this will only show autos that start with "comp" while at
      // competition as defined by the programmer
      autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
         (stream) -> isCompetition
           ? stream.filter(auto -> auto.getName().startsWith("comp"))
           : stream
       );

       SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    driverController.triangle().whileTrue(algaeIntake.voltageCommand(-1));
    driverController.cross().whileTrue(algaeIntake.voltageCommand(1));
    driverController.povUp().whileTrue(climb.voltageClimbCommand(ClimbConstants.kVoltage));
    driverController.povDown().whileTrue(climb.voltageClimbCommand(-ClimbConstants.kVoltage));


    // algae intake/eject
    // driverController.R1().and(algaeIntake.hasAlgae.negate())
    //     .onTrue(algaeIntake
    //         .deployAndIntakeCommand());

    //driverController.triangle().onTrue(algaeIntake.setPositionCommand(45));
    //driverController.triangle().onFalse(algaeIntake.setPositionCommand(0));
    driverController.R1().and(algaeIntake.hasAlgae.negate()).whileTrue(algaeIntake.deployAndIntakeCommand());
    driverController.R1().onFalse(algaeIntake.setPositionCommand(0));
    driverController.R1().and(algaeIntake.hasAlgae).whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage));
    
    algaeIntake.hasAlgae.whileTrue(algaeIntake.intakeVoltageCommand(0.25));

    //r1.and(algaeIntake.hasAlgae.negate()).whileTrue(algaeIntake.deployAndIntakeCommand());
   // r1.onFalse(algaeIntake.setPositionCommand(0));
    //r1.and(algaeIntake.hasAlgae).whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage));
    //r2.whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage));
    //SmartDashboard.put
    //r1.whileTrue(algaeIntake.intakeVoltageCommand(0.25));
    //driverController.cross().onTrue(algaeIntake.setPositionCommand(0));

    // driverController.R1().and(algaeIntake.hasAlgae)
    //     .onTrue(algaeIntake
    //         .intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage)
    //         .alongWith(led.setLEDPatternCommand(LEDConstants.kAlgaeControlledLED)));

    // driverController.R1().onFalse(algaeIntake
    //     .stowCommand());

    // climb
    //operatorController.R2().whileTrue(climb.voltageClimbCommandFactory(ClimbConstants.kVoltage));

    //Coral
    //driverController.triangle().whileTrue(coral.holdAngleCommand(CoralConstants.kIntakePosition));
    //driverController.cross().whileTrue(coral.holdAngleCommand(CoralConstants.kDumpPosition));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();//Commands.runOnce(() -> swerve.drive(new ChassisSpeeds(1, 0, 0), false), swerve);//autoChooser.getSelected();
  }
}

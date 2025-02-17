// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.server.Operation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntakeConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstants;
import frc.robot.Subsystems.Coral.Coral;
import frc.robot.Subsystems.Coral.CoralConstants;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.LED.LEDConstants;

public class RobotContainer {

  // Create controllers
  CommandPS5Controller driverController = new CommandPS5Controller(0);
  CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Create subsystems
  Climb climb = new Climb();
  AlgaeIntake algaeIntake = new AlgaeIntake();
  Coral coral = new Coral();
  LED led = new LED();

  public RobotContainer() {
    configureBindings();

    // defaults coral manipulator to stop
    // coralSubsystem.setDefaultCommand(coralSubsystem.setVoltageCommandFactory(0,0));
    led.setDefaultCommand(led.setLEDPatternCommand(LEDConstants.kIdleLED));

  }

  private void configureBindings() {

    // algae intake/eject
    driverController.R1().and(algaeIntake.hasAlgae.negate())
        .onTrue(algaeIntake
            .deployAndIntakeCommand());

    driverController.R1().and(algaeIntake.hasAlgae)
        .onTrue(algaeIntake
            .intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage)
            .alongWith(led.setLEDPatternCommand(LEDConstants.kAlgaeControlledLED)));

    driverController.R1().onFalse(algaeIntake
        .stowCommand());

    // climb
    operatorController.R2().whileTrue(climb.voltageClimbCommandFactory(ClimbConstants.kVoltage));

    //Coral
    operatorController.triangle().whileTrue(coral.holdAngleCommand(CoralConstants.kIntakePosition));
    operatorController.cross().whileTrue(coral.holdAngleCommand(CoralConstants.kDumpPosition));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

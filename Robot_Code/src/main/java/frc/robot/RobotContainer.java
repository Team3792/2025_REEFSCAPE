// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.AlgaeRemoverSubsystem;
import frc.robot.Subsystems.AlgaeRemoverSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.AlgaeIntakeSubsystem;
import frc.robot.Constants;

public class RobotContainer {

  //Create controllers
  CommandPS5Controller driverController = new CommandPS5Controller(0);
  CommandPS5Controller operatorController = new CommandPS5Controller(1);

  //Create subsystems
  //ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //CoralSubsystem coralSubsystem = new CoralSubsystem();
  AlgaeRemoverSubsystem algaeRemoverSubsystem = new AlgaeRemoverSubsystem();
  // ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  // CoralSubsystem coralSubsystem = new CoralSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  public RobotContainer() {
    configureBindings();
    
    //defaults coral manipulator to stop 
    //coralSubsystem.setDefaultCommand(coralSubsystem.setVoltageCommandFactory(0,0));
    // algaeIntakeSubsystem.setDefaultCommand(algaeIntakeSubsystem.runIntake(0));
  
  }


  private void configureBindings() {

    driverController.triangle().onTrue(algaeRemoverSubsystem.setPositionCommand(180));
    driverController.square().onTrue(algaeRemoverSubsystem.setPositionCommand(90));
    driverController.cross().onTrue(algaeRemoverSubsystem.setPositionCommand(0));

    driverController.R1().and(algaeIntakeSubsystem.hasAlgae.negate())
      .onTrue(algaeIntakeSubsystem.deployAndIntakeCommand());

    driverController.R1().and(algaeIntakeSubsystem.hasAlgae)
      .onTrue(algaeIntakeSubsystem.intakeVoltageCommand(Constants.AlgaeIntakeSubsystem.kEjectVoltage));

    driverController.R1().onFalse(algaeIntakeSubsystem.stowCommand());

    operatorController.cross().onTrue(elevatorSubsystem.setStateCommand(ElevatorSubsystem.ElevatorState.L1));
    operatorController.square().onTrue(elevatorSubsystem.setStateCommand(ElevatorSubsystem.ElevatorState.L2));
    operatorController.triangle().onTrue(elevatorSubsystem.setStateCommand(ElevatorSubsystem.ElevatorState.L1));


    operatorController.povUp().onTrue(elevatorSubsystem.setStateCommand(ElevatorSubsystem.ElevatorState.AlgaeHigh));
    operatorController.povDown().onTrue(elevatorSubsystem.setStateCommand(ElevatorSubsystem.ElevatorState.AlgaeLow));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

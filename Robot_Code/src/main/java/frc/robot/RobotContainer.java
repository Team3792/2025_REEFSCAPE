// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Coral;
import frc.robot.Subsystems.AlgaeRemover;
import frc.robot.Subsystems.AlgaeRemover;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.AlgaeIntake;
import frc.robot.Constants;

public class RobotContainer {

  //Create controllers
  CommandPS5Controller driverController = new CommandPS5Controller(0);
  CommandPS5Controller operatorController = new CommandPS5Controller(1);

  //Create subsystems
  //ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  Coral coralSubsystem = new Coral();
  AlgaeRemover algaeRemoverSubsystem = new AlgaeRemover();
  // ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  // CoralSubsystem coralSubsystem = new CoralSubsystem();
  Elevator elevatorSubsystem = new Elevator();
  AlgaeIntake algaeIntakeSubsystem = new AlgaeIntake();

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

    operatorController.cross().onTrue(elevatorSubsystem.setStateCommand(Elevator.ElevatorState.L1));
    operatorController.square().onTrue(elevatorSubsystem.setStateCommand(Elevator.ElevatorState.L2));
    operatorController.triangle().onTrue(elevatorSubsystem.setStateCommand(Elevator.ElevatorState.L1));


    operatorController.povUp().onTrue(elevatorSubsystem.setStateCommand(Elevator.ElevatorState.AlgaeHigh));
    operatorController.povDown().onTrue(elevatorSubsystem.setStateCommand(Elevator.ElevatorState.AlgaeLow));
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

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
  // ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  Constants constants = new Constants();

  public RobotContainer() {
    configureBindings();
    
    //defaults coral manipulator to stop 
    //coralSubsystem.setDefaultCommand(coralSubsystem.setVoltageCommandFactory(0,0));
    // algaeIntakeSubsystem.setDefaultCommand(algaeIntakeSubsystem.runIntake(0));
  
  }


  private void configureBindings() {
    //driverController.povUp().whileTrue(climbSubsystem.voltageClimbCommandFactory(5));
    //operatorController.R1().whileTrue(coralSubsystem.intakeCommandFactory());

    driverController.triangle().onTrue(algaeRemoverSubsystem.setPositionCommand(180));
    driverController.square().onTrue(algaeRemoverSubsystem.setPositionCommand(90));
    driverController.cross().onTrue(algaeRemoverSubsystem.setPositionCommand(0));
    
    //elevator
  //  operatorController.cross().onTrue(elevatorSubsystem.setPositionCommandFactory(Constants.ElevatorSubsystem.kElevatorL1Position));
  //  operatorController.square().onTrue(elevatorSubsystem.setPositionCommandFactory(Constants.ElevatorSubsystem.kElevatorL2Position));
  //   operatorController.triangle().onTrue(elevatorSubsystem.setPositionCommandFactory(Constants.ElevatorSubsystem.kElevatorL3Position));
    
    //climb
    // driverController.povUp().whileTrue(climbSubsystem.voltageClimbCommandFactory(1));
    // operatorController.povDown().whileTrue(climbSubsystem.voltageClimbCommandFactory(-1));
    // //coral
    // operatorController.R2().whileTrue(coralSubsystem.intakeCommandFactory());
    //algae
    // driverController.R1().and(new Trigger(() -> (algaeIntakeSubsystem.colorSensorV3.getProximity() < 80.0)))
    // .whileTrue(algaeIntakeSubsystem.runIntake(8));
    driverController.R1().whileTrue(algaeIntakeSubsystem.runIntakeCommandFactory(8));
    driverController.L1().whileTrue(algaeIntakeSubsystem.intakeVoltageCommand(-8));
    driverController.R2().onTrue(algaeIntakeSubsystem.setPositionCommand(Constants.AlgaeIntakeSubsystem.kAlgaeIntakePosition));
    driverController.L2().onTrue(algaeIntakeSubsystem.setPositionCommand(Constants.AlgaeIntakeSubsystem.kAlgaeOuttakePosition));
    


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

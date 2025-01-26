// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Subsystems.AlgaeRemoverSubsystem;
import frc.robot.Subsystems.AlgaeRemoverSubsystem;
import frc.robot.Subsystems.ClimbSubsystem;

public class RobotContainer {

  //Create controllers
  CommandPS5Controller driverController = new CommandPS5Controller(0);
  CommandPS5Controller operatorController = new CommandPS5Controller(1);

  //Create subsystems
  //ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //CoralSubsystem coralSubsystem = new CoralSubsystem();
  AlgaeRemoverSubsystem algaeRemoverSubsystem = new AlgaeRemoverSubsystem();

  public RobotContainer() {
    configureBindings();
    
    //defaults coral manipulator to stop 
    //coralSubsystem.setDefaultCommand(coralSubsystem.setVoltageCommandFactory(0,0));
  }


  private void configureBindings() {
    //driverController.povUp().whileTrue(climbSubsystem.voltageClimbCommandFactory(5));
    //operatorController.R1().whileTrue(coralSubsystem.intakeCommandFactory());

    driverController.triangle().onTrue(algaeRemoverSubsystem.setPositionCommand(180));
    driverController.square().onTrue(algaeRemoverSubsystem.setPositionCommand(90));
    driverController.cross().onTrue(algaeRemoverSubsystem.setPositionCommand(0));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

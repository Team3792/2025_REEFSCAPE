// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class RobotContainer {
  CommandPS5Controller operator = new CommandPS5Controller(0);
  public RobotContainer() {
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    configureBindings();

  }

  private void configureBindings() {
    //set position of elevator
    operator.povUp().whileTrue(ElevatorSubsystem.goToPositionCommandFactory(kElevatorLevel1Position));
    operator.povRight().whileTrue(ElevatorSubsystem.goToPositionCommandFactory(kEkevatorLevel2Position));
    operator.povDown().whileTrue(ElevatorSubsystem.goToPositionCommandFactory(kElevatorLevel3Position));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

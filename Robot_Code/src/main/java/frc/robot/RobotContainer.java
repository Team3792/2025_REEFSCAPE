// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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

  PowerDistribution pdh = new PowerDistribution(HardwareMap.kPDH, ModuleType.kRev);

  // Create controllers
  CommandPS5Controller controller = new CommandPS5Controller(HardwareMap.kControllerPort);

  // Create subsystems
  Climb climb = new Climb();
  AlgaeIntake algaeIntake = new AlgaeIntake();
  Coral coral = new Coral();
  LED led = new LED();
  Swerve swerve = new Swerve();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    pdh.clearStickyFaults();

    NamedCommands.registerCommand("IntakePosition",
        coral.holdAngleCommand(CoralConstants.kIntakePosition).withTimeout(CoralConstants.kAutoIntakeTime));
    NamedCommands.registerCommand("DumpPosition",
        coral.holdAngleCommand(CoralConstants.kDumpPosition).withTimeout(CoralConstants.kAutoDumpTime));
    
        swerve.setDefaultCommand(
          new DriveCommand(swerve, 
            () -> -controller.getLeftY(), 
            () -> -controller.getLeftX(), 
            () -> -controller.getRightX()));
                                                                                                    
    configureBindings();
   
    led.setDefaultCommand(led.setLEDPatternCommand(LEDConstants.kIdleLED));

    // Auto builder
    boolean isCompetition = false;
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    //Climb
    controller.povUp().whileTrue(climb.voltageClimbCommand(ClimbConstants.kUpVoltage));
    controller.povDown().whileTrue(climb.voltageClimbCommand(ClimbConstants.kDownVoltage));

   //Algae
    controller.R1().whileTrue(algaeIntake.deployAndIntakeCommand());
    controller.R1().onFalse(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kStowPosition));
    controller.L1().whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage));

    algaeIntake.hasAlgae.whileTrue(algaeIntake.intakeVoltageCommand(0.5));

    // Coral
    controller.triangle().whileTrue(coral.holdAngleCommand(CoralConstants.kIntakePosition));
    controller.square().whileTrue(coral.holdAngleCommand(CoralConstants.kMidPosition));
    controller.cross().whileTrue(coral.holdAngleCommand(CoralConstants.kDumpPosition));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

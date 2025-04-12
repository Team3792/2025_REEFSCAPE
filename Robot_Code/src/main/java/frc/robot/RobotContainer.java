// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Parameter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntake;
import frc.robot.Subsystems.AlgaeIntake.AlgaeIntakeConstants;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstants;
import frc.robot.Subsystems.Coral.Coral;
import frc.robot.Subsystems.Coral.CoralConstants;
import frc.robot.Subsystems.Swerve.AlignToTagCommand;
import frc.robot.Subsystems.Swerve.ManualDriveCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Swerve.AlignToTagCommand.AlignType;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.LED.LEDConstants;

public class RobotContainer {

  PowerDistribution pdh = new PowerDistribution(HardwareMap.kPDH, ModuleType.kRev);

  // Create controllers
  CommandPS5Controller driver = new CommandPS5Controller(HardwareMap.kDriverPort);
  CommandPS5Controller operator = new CommandPS5Controller(HardwareMap.kOperatorPort);

  // Create subsystems
  Swerve swerve = new Swerve();
  Climb climb = new Climb();
  AlgaeIntake algaeIntake = new AlgaeIntake();
  Coral coral = new Coral();
  LED led = new LED();
  

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    SmartDashboard.putNumber("DriveVoltage", 0);

    pdh.clearStickyFaults();
    // SmartDashboard.putData("PDH", pdh);

    NamedCommands.registerCommand("IntakePosition",
        coral.holdAngleCommand(CoralConstants.kIntakePosition, led).withTimeout(CoralConstants.kAutoIntakeTime));
    NamedCommands.registerCommand("Prime Position", coral.setAngleCommand(60));
    NamedCommands.registerCommand("Down", coral.setAngleCommand(100));
    NamedCommands.registerCommand("Up", coral.setAngleCommand(CoralConstants.kIntakePosition));
    NamedCommands.registerCommand("DumpPosition",
        coral.holdAngleCommand(110, led).withTimeout(CoralConstants.kAutoDumpTime));
    NamedCommands.registerCommand("Climb Down", climb.toPositionCommand(3, 90));

    climb.setNeutralMode(NeutralModeValue.Brake);
    configureDriverBindings(driver);
    configureOperatorBindings(operator);

    led.setDefaultCommand(led.idleOrErrorCommand());

    // Auto builder
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> MatchData.kIsCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp") || auto.getName().startsWith("exp"))
            : stream);

    // Absolute back up auto ---- Drive forward at voltage (literal constants here)
    autoChooser.addOption("***Drive Forward***", swerve.driveForwardCommand(1.0, 2.0));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverBindings(CommandPS5Controller controller) {
    // Algae
    controller.R1().onTrue(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kAlgaeIntakePosition));
    controller.R1().onFalse(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kStowPosition));

    controller.R1()
      .and(algaeIntake.hasAlgae.negate())
      //.and(swerve.isTipped.negate())
      .whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kIntakeVoltage, led));

    // controller.R1().and(swerve.isTipped).whileTrue(
    //   algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage, led)
    //   .alongWith(swerve.stopCommand())
    //   );
    algaeIntake.setDefaultCommand(algaeIntake.getHoldCommand());

    controller.L1().whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage, led));


    // Swerve
    swerve.setDefaultCommand(
        new ManualDriveCommand(swerve,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> controller.L3().getAsBoolean(),
            () -> controller.R3().getAsBoolean()));

    controller.options().onTrue(Commands.runOnce(() -> swerve.resetHeading(), swerve));

    //Auto aligning
    controller.R2().whileTrue(
      new ParallelCommandGroup(new AlignToTagCommand(swerve, AlignType.CoralStation, SwerveConstants.kCenterAlign), coral.holdAngleCommand(CoralConstants.kIntakePosition, led)));
    controller.L2().whileTrue(new AlignToTagCommand(swerve, AlignType.Reef, SwerveConstants.kLeftAlign));
  }

  private void configureOperatorBindings(CommandPS5Controller controller) {
    // Coral
    controller.triangle().whileTrue(coral.holdAngleCommand(CoralConstants.kIntakePosition, led));
    controller.square().whileTrue(coral.holdAngleCommand(CoralConstants.kMidPosition, led));
    controller.cross().whileTrue(coral.holdAngleCommand(CoralConstants.kDumpPosition, led));

    // Climb
    controller.povUp().whileTrue(climb.voltageClimbCommand(ClimbConstants.kUpVoltage));
    controller.povDown().whileTrue(climb.voltageClimbCommand(ClimbConstants.kDownVoltage));


    //Manual Algae controler
    controller.R2().whileTrue(algaeIntake.voltageCommand(AlgaeIntakeConstants.kManualVoltage));
    controller.L2().whileTrue(algaeIntake.voltageCommand(-AlgaeIntakeConstants.kManualVoltage));
    controller.options().onTrue(algaeIntake.manualModeCommand(led, coral));

    controller.R2().whileTrue(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kCoralGroundIntakePosition)
        .alongWith(algaeIntake.runRolllerCommand(3)));
        
    controller.L2().whileTrue(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kCoralEjectPosition)
        .alongWith(algaeIntake.runRolllerCommand(-3)));
  }
  public void initiateBrakes(){
    climb.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command getAutonomousCommand() {
    return Commands.parallel(climb.toPositionCommand(0, 90), autoChooser.getSelected());
  }
}

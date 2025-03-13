// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class RobotContainer {

  PowerDistribution pdh = new PowerDistribution(HardwareMap.kPDH, ModuleType.kRev);

  // Create controllers
  CommandPS5Controller driver = new CommandPS5Controller(HardwareMap.kDriverPort);
  CommandPS5Controller operator = new CommandPS5Controller(HardwareMap.kOperatorPort);

  // Create subsystems
  Climb climb = new Climb();
  AlgaeIntake algaeIntake = new AlgaeIntake();
  Coral coral = new Coral();
  //LED led = new LED();
  public Swerve swerve = new Swerve();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    SmartDashboard.putNumber("DriveVoltage", 0);

    pdh.clearStickyFaults();
    //SmartDashboard.putData("PDH", pdh);

    NamedCommands.registerCommand("IntakePosition",
        coral.holdAngleCommand(CoralConstants.kIntakePosition).withTimeout(CoralConstants.kAutoIntakeTime));
    NamedCommands.registerCommand("DumpPosition",
        coral.holdAngleCommand(100).withTimeout(CoralConstants.kAutoDumpTime));
    
        
        // swerve.setDefaultCommand(
        //   new FunctionalCommand(
        //     () -> {}, 
        //     () -> swerve.driveForwardVoltage(SmartDashboard.getNumber("DriveVoltage", 0)), 
        //     (a) -> {}, 
        //     () -> false, 
        //     swerve)
        //   );
                                                                                                    
    configureDriverBindings(driver);
    configureOperatorBindings(operator);
   
    //led.setDefaultCommand(led.setLEDPatternCommand(LEDConstants.kIdleLED));

    // Auto builder
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> MatchData.kIsCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream);

    //Absolute back up auto ---- Drive forward at voltage (literal constants here)
    autoChooser.addOption("***Drive Forward***", swerve.driveForwardCommand(1.0, 2.0));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverBindings(CommandPS5Controller controller){
    //Algae
    controller.R1().onTrue(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kAlgaeIntakePosition));
    controller.R1().onFalse(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kStowPosition));
    controller.R1().and(algaeIntake.hasAlgae.negate()).whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kIntakeVoltage));
    algaeIntake.setDefaultCommand(algaeIntake.getHoldCommand());
    //algaeIntake.hasAlgae.whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kHoldingVoltage));
    

    //Swerve

    swerve.setDefaultCommand(
          new ManualDriveCommand(swerve, 
            () -> -controller.getLeftY(), 
            () -> -controller.getLeftX(), 
            () -> -controller.getRightX(),
            () -> controller.L3().getAsBoolean(),
            () -> controller.R3().getAsBoolean()));

    controller.cross().whileTrue(
      new AlignToTagCommand(swerve, SwerveConstants.kCenterAlign)
    );

    controller.options().onTrue(Commands.runOnce(() -> swerve.resetHeading(),  swerve));
  }

  private void configureOperatorBindings(CommandPS5Controller controller){
    //Coral
    controller.triangle().whileTrue(coral.holdAngleCommand(CoralConstants.kIntakePosition));
    controller.square().whileTrue(coral.holdAngleCommand(CoralConstants.kMidPosition));
    controller.cross().whileTrue(coral.holdAngleCommand(CoralConstants.kDumpPosition)); 

    //Climb
    controller.povUp().whileTrue(climb.voltageClimbCommand(ClimbConstants.kUpVoltage));
    controller.povDown().whileTrue(climb.voltageClimbCommand(ClimbConstants.kDownVoltage));

    controller.R2().whileTrue(algaeIntake.voltageCommand(1));
    controller.L2().whileTrue(algaeIntake.voltageCommand(-1));

    controller.R1().whileTrue(algaeIntake.deployAndIntakeCommand());
    controller.R1().onFalse(algaeIntake.setPositionCommand(AlgaeIntakeConstants.kStowPosition));
    controller.L1().whileTrue(algaeIntake.intakeVoltageCommand(AlgaeIntakeConstants.kEjectVoltage));
   
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

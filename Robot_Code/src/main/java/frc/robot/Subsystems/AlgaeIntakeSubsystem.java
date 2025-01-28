// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import netscape.javascript.JSException;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  public algaeIntakeState state = algaeIntakeState.noAlgae;
  TalonFX algaeRotate = new TalonFX(Constants.HardwareAddresses.kAlgaeRotateID);
  SparkMax algaeSpin = new SparkMax(Constants.HardwareAddresses.kAlgaeSpinID, MotorType.kBrushless);

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);

  I2C.Port i2c = I2C.Port.kOnboard;
    public ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2c);

  public AlgaeIntakeSubsystem() {
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig.smartCurrentLimit(40, 40);
    algaeSpin.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.Slot0.kG = Constants.AlgaeIntakeSubsystem.kG;
    talonConfig.Slot0.kP = Constants.AlgaeIntakeSubsystem.kP;
    talonConfig.Slot0.kI = Constants.AlgaeIntakeSubsystem.kI;
    talonConfig.Slot0.kD = Constants.AlgaeIntakeSubsystem.KD;

    algaeRotate.getConfigurator().apply(talonConfig);

  }

  public enum algaeIntakeState{
    noAlgae,
    yesAlgae,
    intake,
    outtake
  }
  //returns true when there is algae in manipulator
  public boolean isAlgae(){
    return colorSensorV3.getProximity() > 80.0;
  }
  
  public void setVoltage(double voltage){
    algaeSpin.setVoltage(voltage);
  }
  //applies voltage to algaeSpin motor
  public Command intakeVoltageCommand(double voltage){
    return Commands.startEnd(()->{setVoltage(voltage);}, ()->{setVoltage(0);}, this);
  }
  
  public Command runIntakeCommandFactory(double voltage){
    return new SequentialCommandGroup(
      this.runOnce(() -> {setVoltage(voltage);}),
      Commands.waitUntil(this::isAlgae),
      intakeVoltageCommand(0)
    );
  }
  public void setPosition(double setPoint){
    algaeRotate.setControl(positionControl.withPosition(setPoint));
  }
  
  public Command setPositionCommand(double position){
    return this.runOnce(()-> {setPosition(position);});
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae current", algaeSpin.getOutputCurrent());
    SmartDashboard.putNumber("Algae speed", algaeSpin.getEncoder().getVelocity());
    SmartDashboard.putNumber("Distance", colorSensorV3.getProximity());
  }
}

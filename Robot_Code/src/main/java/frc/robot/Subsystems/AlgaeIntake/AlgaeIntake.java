// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.HardwareMap;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFX pivot = new TalonFX(HardwareMap.kAlgaeRotate);
  SparkMax drive = new SparkMax(HardwareMap.kAlgaeSpin, MotorType.kBrushless);

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2cPort);

  public Trigger hasAlgae = new Trigger(this::hasAlgae);

  public AlgaeIntake() {
    //Configure drive motor
    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig.smartCurrentLimit(40, 40);
    drive.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Configure pivot motor
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.Slot0.kG = AlgaeIntakeConstants.kG;
    talonConfig.Slot0.kP = AlgaeIntakeConstants.kP;
    talonConfig.Slot0.kI = AlgaeIntakeConstants.kI;
    talonConfig.Slot0.kD = AlgaeIntakeConstants.kD;

    pivot.getConfigurator().apply(talonConfig);
  }

  //Returns true when there is algae in manipulator
  public boolean hasAlgae(){
    return colorSensorV3.getProximity() > AlgaeIntakeConstants.kProximityMin;
  }
  
  public void setDriveVoltage(double voltage){
    drive.setVoltage(voltage);
  }

  //applies voltage to drive motor
  public Command intakeVoltageCommand(double voltage){
    return Commands.startEnd(()->{setDriveVoltage(voltage);}, ()->{setDriveVoltage(0);}, this);
  }


  //Deploys and runs intake until algae is detected
  public Command deployAndIntakeCommand(){
    return setPositionCommand(AlgaeIntakeConstants.kAlgaeIntakePosition)
          .alongWith(intakeVoltageCommand(AlgaeIntakeConstants.kIntakeVoltage))
          .onlyWhile(hasAlgae.negate());
  }

  //Stows and stops intake
  public Command stowCommand(){
    return setPositionCommand(AlgaeIntakeConstants.kStowPosition)
          .alongWith(this.runOnce(() -> {setDriveVoltage(0);}));
  }

  public void setPosition(double setPoint){
    pivot.setControl(positionControl.withPosition(setPoint));
  }
  
  public Command setPositionCommand(double position){
    return this.runOnce(()-> {setPosition(position);});
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae current", drive.getOutputCurrent());
    SmartDashboard.putNumber("Algae speed", drive.getEncoder().getVelocity());
    SmartDashboard.putNumber("Distance", colorSensorV3.getProximity());
  }
}

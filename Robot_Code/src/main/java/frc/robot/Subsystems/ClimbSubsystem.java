// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbSubsystem extends SubsystemBase {
  //Define hardware
  TalonFX leftMotor = new TalonFX(Constants.HardwareAddresses.climbLeftMotorID);
  TalonFX rightMotor = new TalonFX(Constants.HardwareAddresses.climbRightMotorID);

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);


  public ClimbSubsystem() {
    //Motor configuration
    
    TalonFXConfiguration climbMotorConfiguration = new TalonFXConfiguration();
    climbMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotorConfiguration.Slot0.kG = Constants.ClimbSubsystem.kG;
    climbMotorConfiguration.Slot0.kP = Constants.ClimbSubsystem.kP;
    climbMotorConfiguration.Slot0.kI = Constants.ClimbSubsystem.kI;
    climbMotorConfiguration.Slot0.kD = Constants.ClimbSubsystem.kD;

    leftMotor.getConfigurator().apply(climbMotorConfiguration);
    rightMotor.getConfigurator().apply(climbMotorConfiguration);
  }

  public void setPosition(double leftDegrees, double rightDegrees){
    leftMotor.setControl(positionControl.withPosition(leftDegrees));
    rightMotor.setControl(positionControl.withPosition(rightDegrees));
  }


  //Apply voltage to right and left motors; useful for testing
  private void applyVoltage(double leftVoltage, double rightVoltage){
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  public Command voltageClimbCommandFactory(double voltage){
    return Commands.startEnd(
      () -> {applyVoltage(voltage, voltage);}, 
      () -> {applyVoltage(0, 0);},
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Telemetry
    SmartDashboard.putNumber("subsystems/climb/right position", rightMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("subsystems/climb/left position", leftMotor.getPosition().getValueAsDouble());
  }
}

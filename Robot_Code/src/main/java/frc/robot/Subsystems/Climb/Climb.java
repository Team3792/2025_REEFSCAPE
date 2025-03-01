// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase {
  //Define hardware
  TalonFX leftMotor = new TalonFX(HardwareMap.kclimbLeft);
  TalonFX rightMotor = new TalonFX(HardwareMap.kclimbRight);

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);


  public Climb() {
    //Motor configuration
    
    TalonFXConfiguration climbMotorConfiguration = new TalonFXConfiguration();
    climbMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbMotorConfiguration.Slot0.kG = ClimbConstants.kG;
    climbMotorConfiguration.Slot0.kP = ClimbConstants.kP;
    climbMotorConfiguration.Slot0.kI = ClimbConstants.kI;
    climbMotorConfiguration.Slot0.kD = ClimbConstants.kD;

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

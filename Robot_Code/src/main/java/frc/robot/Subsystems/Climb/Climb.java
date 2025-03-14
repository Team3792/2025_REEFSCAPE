// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.Util.CANManager;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase {
  //Define hardware
  TalonFX leftMotor = new TalonFX(HardwareMap.kclimbLeft.id());
  TalonFX rightMotor = new TalonFX(HardwareMap.kclimbRight.id());

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);


  public Climb() {
    //Motor configuration
    
    TalonFXConfiguration config = ClimbConstants.getConfig();

    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);
    zeroPosition();

    //Connection
    CANManager.addConnection(HardwareMap.kclimbLeft, leftMotor);
    CANManager.addConnection(HardwareMap.kclimbRight, rightMotor);
    // ConnectionAlert.createConnection("Left Climb", leftMotor::isConnected);
    // ConnectionAlert.createConnection("Right Climb", rightMotor::isConnected);
  }

  public void zeroPosition(){
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  public void setNeutralMode(NeutralModeValue neutralMode){
    leftMotor.setNeutralMode(neutralMode);
    rightMotor.setNeutralMode(neutralMode);
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

  public Command voltageClimbCommand(double voltage){
    return Commands.startEnd(
      () -> {applyVoltage(voltage, voltage);}, 
      () -> {applyVoltage(0, 0);},
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Telemetry
    SmartDashboard.putNumber("Subsystems/Climb/Front Position", rightMotor.getPosition().getValueAsDouble()*ClimbConstants.kRotationsToDegrees);
    SmartDashboard.putNumber("Subsystems/Climb/Back Position", leftMotor.getPosition().getValueAsDouble()*ClimbConstants.kRotationsToDegrees);
  }
}

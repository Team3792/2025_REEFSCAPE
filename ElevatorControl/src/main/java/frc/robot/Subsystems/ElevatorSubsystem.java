// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.net.Authenticator.RequestorType;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  TalonFX rightMotor = new TalonFX(0);
  TalonFX leftMotor = new TalonFX(0);
  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);

  public ElevatorSubsystem() {
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

  }

  //use for testing elevator
  public void setVoltage(double voltageLeft){
    leftMotor.setVoltage(voltageLeft);
  }

  public void setPosition(double setPoint){
    leftMotor.setControl(positionControl.withPosition(setPoint).withFeedForward(kElevatorFeedForwardValue));
  }
  
  public Command setPositionCommandFactory(double position){
    return this.runOnce(()-> {setPosition(position);});
  }

  //public Command goToPositionCommandFactory(double level){
  //   //LED.setPower();
  //   return this.runOnce(()-> {setPositionCommandFactory(level);});
  // }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  //define hardware
  SparkMax leftMotor = new SparkMax(Constants.HardwareAddresses.coralLeftMotorID, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(Constants.HardwareAddresses.coralRightMotorID, MotorType.kBrushless);

  AnalogInput frontSwitch = new AnalogInput(Constants.HardwareAddresses.frontSwitchID);
  AnalogInput backSwitch = new AnalogInput(Constants.HardwareAddresses.backSwitchID);

  public Trigger hasCoral = new Trigger(this::getBackSwitch);

  public CoralSubsystem() {}

  public boolean getFrontSwitch(){
    return frontSwitch.getVoltage() > Constants.CoralSubsystem.kswitchVoltageThreshold; 
  }

  public boolean getBackSwitch (){
    return backSwitch.getVoltage() > Constants.CoralSubsystem.kswitchVoltageThreshold; 
  }

  public void setVoltage(double leftVoltage, double rightVoltage){
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  public Command setVoltageCommandFactory(double leftVoltage, double rightVoltage){
    return this.runOnce(()-> {setVoltage(leftVoltage, rightVoltage);});
  }

  public Command intakeCommand(){
    return new SequentialCommandGroup(
      setVoltageCommandFactory(5, 5), //Fast
      Commands.waitUntil(this::getFrontSwitch),
      setVoltageCommandFactory(1, 1), //Slow
      Commands.waitUntil(this::getBackSwitch),
      setVoltageCommandFactory(0, 0) //Stop
    );
  }

 
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("subsystem/coral/frontswitch", getFrontSwitch());
    SmartDashboard.putBoolean("subsystem/coral/backswitch", getBackSwitch());

  }
}

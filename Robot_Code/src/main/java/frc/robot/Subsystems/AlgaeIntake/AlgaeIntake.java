// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Coral.Coral;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.LED.LEDConstants;
import frc.robot.HardwareMap;
import frc.robot.Util.CANManager;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFX pivot = new TalonFX(HardwareMap.kAlgaeRotate.id());
  SparkMax drive = new SparkMax(HardwareMap.kAlgaeSpin.id(), MotorType.kBrushless);

  private ProfiledPIDController pidController = AlgaeIntakeConstants.pivotPIDConfig.getController();

  
  public AlgaeIntake() {
    //Configure motors
    drive.configure(AlgaeIntakeConstants.getDriveConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.getConfigurator().apply(AlgaeIntakeConstants.getPivotConfig());

    drive.getEncoder().setPosition(0);
    pivot.setPosition(angleToPosition(90)); //TODO: change to absolute encoder
    pidController.setGoal(90);
    pidController.reset(getAngleDegrees()); //Reset position to current angle to generate profile to return to 0 at start

    CANManager.addConnection(HardwareMap.kAlgaeRotate, pivot);
  }



  public Command closeScissorsCommand(){
    
    return this.run(() -> {drive.setVoltage(-AlgaeIntakeConstants.kCuttingVoltage);}).onlyWhile(() -> (drive.getEncoder().getPosition() > 0.5)).andThen(this.runOnce(() -> drive.setVoltage(-0.5)));
  }
  public Command openScissorsCommand(){
    return this.run(() -> {drive.setVoltage(AlgaeIntakeConstants.kCuttingVoltage);}).onlyWhile(() -> (drive.getEncoder().getPosition() <9)).andThen(this.runOnce(() -> drive.setVoltage(0)));
  }

  private double getAngleDegrees(){
    return pivot.getPosition().getValueAsDouble()/AlgaeIntakeConstants.kPivotRatio * 360;
  }

  private double angleToPosition(double angleDegrees){
    return angleDegrees/360.0 * AlgaeIntakeConstants.kPivotRatio;
  }


  //Stows and stops intake
  public Command stowCommand(){
    return setPositionCommand(AlgaeIntakeConstants.kStowPosition)
          .alongWith(closeScissorsCommand());
  }

  public void setPosition(double setPointDegrees){
    pidController.setGoal(setPointDegrees);
  }
  
  public Command setPositionCommand(double position){
    return Commands.runOnce(() -> {setPosition(position);});
  }

  

  private void runToPosition(){
    double gravityFF = Math.cos(getAngleDegrees() * Math.PI / 180.0) * AlgaeIntakeConstants.kG;
    double velocityFF = AlgaeIntakeConstants.kVelocityFF * pidController.getSetpoint().velocity;
    double pidOutput = pidController.calculate(getAngleDegrees());
    pivot.setVoltage(pidOutput + gravityFF + velocityFF);
  }
  
  @Override
  public void periodic() {
    runToPosition();
    SmartDashboard.putNumber("Algae Drive Position", drive.getEncoder().getPosition());
    SmartDashboard.putNumber("Algae pivot position", getAngleDegrees());
  }
}

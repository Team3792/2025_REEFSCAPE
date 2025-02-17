// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Coral;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
  SparkMax pivot = new SparkMax(HardwareMap.kCoralPivot, MotorType.kBrushless);
  AbsoluteEncoder absoluteEncoder;

  ProfiledPIDController pidController = CoralConstants.kPivotPID.getController();

  public Coral() {
    //Configure motor
    pivot.configure(CoralConstants.pivotConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    absoluteEncoder = pivot.getAbsoluteEncoder();
  }


  //Command that sets angle once
  public Command setAngleCommand(double setPointDegrees){
    return this.runOnce(() -> pidController.setGoal(setPointDegrees));
  }

  //Command that set angle and then returns to 0
  public Command holdAngleCommand(double setPointDegrees){
    return Commands.startEnd(
      () -> pidController.setGoal(setPointDegrees), 
      () -> pidController.setGoal(0), 
      this);
  }

  //Returns the tilted forward angle of the bucket where 0 is vertical
  public double getAngleDegrees(){
    return absoluteEncoder.getPosition();
  }

  private void runToPosition(){
    double gravityFF = -Math.sin(getAngleDegrees() * Math.PI / 180.0);
    double velocityFF = CoralConstants.kVelocityFF * pidController.getSetpoint().velocity;
    double pidOutput = pidController.calculate(getAngleDegrees());

    pivot.setVoltage(pidOutput + gravityFF + velocityFF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runToPosition();
  }
}

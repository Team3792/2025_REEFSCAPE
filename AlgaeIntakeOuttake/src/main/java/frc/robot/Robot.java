// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.introspect.AnnotatedMethodMap;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  // motors
  PWMSparkMax algaeControlMotor = new PWMSparkMax(10);

  PWMSparkMax coralControlMotorRight = new PWMSparkMax(11);
  PWMSparkMax coralControlMotorLeft = new PWMSparkMax(12);

  TalonFX elevatorMotorLeft = new TalonFX(1);
  TalonFX elevatorMotorRight = new TalonFX(2);

  TalonFX climbMotorRight = new TalonFX(3);
  TalonFX climbMotorLeft = new TalonFX(4);

  // controllers
  PS5Controller driver = new PS5Controller(0);
  PS5Controller operator = new PS5Controller(1);

  //LED
  Spark ledSpark = new Spark(0);

  public Robot() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putData(ledSpark); //Add spark to smart dashboard
  }

  @Override
  public void teleopPeriodic() {
    // driver controls
    if (driver.getR1ButtonPressed()) {
      algaeControlMotor.setVoltage(3);
    } else if (driver.getL1ButtonPressed()) {
      algaeControlMotor.setVoltage(-3);
    } else {
      algaeControlMotor.setVoltage(0);
    }

    // operator controls
    if (operator.getR1ButtonPressed()) {
      coralControlMotorLeft.setVoltage(3);
      coralControlMotorRight.setVoltage(3);
    } else if (operator.getL1ButtonPressed()) {
      coralControlMotorLeft.setVoltage(-3);
      coralControlMotorRight.setVoltage(-3);
    } else {
      coralControlMotorLeft.setVoltage(0);
      coralControlMotorRight.setVoltage(0);
    }

    if (operator.getR2ButtonPressed()) {
      elevatorMotorLeft.setVoltage(3);
      elevatorMotorRight.setVoltage(3);
    } else if (operator.getL2ButtonPressed()) {
      elevatorMotorLeft.setVoltage(-3);
      elevatorMotorRight.setVoltage(-3);
    } else {
      elevatorMotorLeft.setVoltage(0);
      elevatorMotorRight.setVoltage(0);
    }

    if (operator.getR3ButtonPressed()) {
      climbMotorLeft.setVoltage(3);
      climbMotorRight.setVoltage(3);
    } else if (operator.getL3ButtonPressed()) {
      climbMotorLeft.setVoltage(-3);
      climbMotorRight.setVoltage(-3);
    } else {
      climbMotorLeft.setVoltage(0);
      climbMotorRight.setVoltage(0);
    }

  }
  // coral control

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

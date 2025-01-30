// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeRemover;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class AlgaeRemover extends SubsystemBase {
  /** Creates a new AlgaeRemover. */
  SparkMax pivotMotor = new SparkMax(HardwareMap.kAlgaeRemover, MotorType.kBrushless);
  SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  public AlgaeRemover() {
    //Configure motors
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    
    config.encoder
      .positionConversionFactor(360.0/AlgaeRemoverConstants.kGearRatio);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.005, 0.0, 0.0); 
  

      pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      pivotMotor.getEncoder().setPosition(0);

      config.smartCurrentLimit(20, 40);
    }

  public Command setPositionCommand(double positionDegrees){
    System.out.println("Going to: " + positionDegrees);
    //return Commands.print("Going to: " + positionDegrees);
    return this.runOnce(() -> {pivotController.setReference(positionDegrees, ControlType.kPosition);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("subsystems/AlgaeRemover/Position", pivotMotor.getEncoder().getPosition());
  }
}

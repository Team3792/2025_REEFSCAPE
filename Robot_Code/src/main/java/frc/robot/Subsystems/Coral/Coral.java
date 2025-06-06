// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Coral;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.LED.LEDConstants;
import frc.robot.Util.CANManager;


public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
  TalonFX pivot = new TalonFX(HardwareMap.kCoralPivot.id());
  DutyCycleEncoder encoder = new DutyCycleEncoder(HardwareMap.kCoralEncoder);

  ProfiledPIDController pidController = CoralConstants.kPivotPID.getController();

  public Coral() {
    //Configure motor
    pivot.getConfigurator().apply(CoralConstants.pivotConfig());

    pidController.reset(getAngleDegrees());
    pidController.setGoal(0);
    pidController.enableContinuousInput(0, 360.0);
    CANManager.addConnection(HardwareMap.kCoralPivot, pivot);
  }


  //Command that sets angle once
  public Command setAngleCommand(double setPointDegrees){
    return this.runOnce(() -> pidController.setGoal(setPointDegrees));
  }

  public void setAngle(double setPointDegrees){
    pidController.setGoal(setPointDegrees);
  }

  //Command that set angle and then returns to 0
  public Command holdAngleCommand(double setPointDegrees, LED led){
    return Commands.startEnd(
      () -> 
        {
          pidController.setGoal(setPointDegrees);
          led.setPattern(LEDConstants.kCoralManipulation);
        }, 
      () -> pidController.setGoal(0), 
      this, led);
  }

  //Returns the tilted forward angle of the bucket where 0 is vertical
  public double getAngleDegrees(){
    return (-360 * (encoder.get() - CoralConstants.kEncoderOffset) + 360) % 360;
  }

  private void runToPosition(){
    double gravityFF = -Math.sin(getAngleDegrees() * Math.PI / 180.0) * CoralConstants.kGravityFF;
    double velocityFF = CoralConstants.kVelocityFF * pidController.getSetpoint().velocity;
    double pidOutput = pidController.calculate(getAngleDegrees());

    pivot.setVoltage(pidOutput + gravityFF + velocityFF);
    SmartDashboard.putNumber("Coral/velocity", pivot.getVelocity().getValueAsDouble()/15*360.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runToPosition();
    SmartDashboard.putNumber("Subsystems/Coral", getAngleDegrees());
  }
}

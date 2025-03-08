// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Dealgaefier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dealgaefier extends SubsystemBase {
  /** Creates a new Dealgaefier. */
  TalonFX dealgaefier = new TalonFX(0);
  DutyCycleEncoder dealgaefierEncoder = new DutyCycleEncoder(0);
  ProfiledPIDController pidController = DealgaefierConstants.kDealgaefierPID.getController();

  public Dealgaefier() {
    dealgaefier.getConfigurator().apply(DealgaefierConstants.dealgaefierConfig());

    pidController.reset(getAngleDegrees());
  }

  public double getAngleDegrees(){
    return (-360 * (dealgaefierEncoder.get() - DealgaefierConstants.kDealgaefierEncoderOffset) + 360) % 360;
  }

 
  public void applyVoltage(double voltage) {
    dealgaefier.setVoltage(voltage);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

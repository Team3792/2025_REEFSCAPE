// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  Spark LED = new Spark(60);
  public LED() {

  }
  private void LEDPatterns(double LEDPattern){
    LED.set(LEDPattern);
  }
  public Command setLEDPatternCommand(double LEDPattern){
    return this.runOnce(()-> {LEDPatterns(LEDPattern);});
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Subsystems/LED/LED/LEDPatterns", LED.getVoltage())
  }
}

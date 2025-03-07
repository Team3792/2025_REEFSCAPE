// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  Spark led = new Spark(HardwareMap.kLED);
  
  public LED() {}

  private void LEDPatterns(double LEDPattern){
    led.set(LEDPattern);
  }
  public Command setLEDPatternCommand(double LEDPattern){
    return this.runOnce(()-> {LEDPatterns(LEDPattern);});
    //return Commands.none();
  }

  @Override
  public void periodic() {}
}

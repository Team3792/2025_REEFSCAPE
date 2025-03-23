// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.MatchData;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  Spark led = new Spark(HardwareMap.kLED);
  
  public LED() {}

  public void setPattern(double ledPattern){
    led.set(ledPattern);
  }
  
  public Command instantSetLEDPatternCommand(double ledPattern){
    return this.runOnce(()-> setPattern(ledPattern));
    //return Commands.none();
  }

  public Command whileSetLEDPatternCommand(double ledPattern){
    return this.startEnd(
      () -> setPattern(ledPattern), 
      () -> {});
  }

  public Command idleOrErrorCommand(){
    return this.run(
      () -> {
        if(MatchData.error){
          setPattern(LEDConstants.kIdle);
        } else {
          setPattern(LEDConstants.kError);
        }
      });
  }

  @Override
  public void periodic() {}
}

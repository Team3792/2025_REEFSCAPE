// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   //algae control
    PWMSparkMax algaeControlMotor = new PWMSparkMax(0);
    
    //neos for coral man
    PWMSparkMax coralControlMotorRight = new PWMSparkMax(0);
    PWMSparkMax coralControlMotorLeft = new PWMSparkMax(0);

    //falcon for rest

    //elevator
     elevatorRight = new PWMSparkMax(0);
    PWMSparkMax elevatorLeft = new PWMSparkMax(0);

    PS5Controller controller = new PS5Controller(0);


  public Robot() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    //algae control (in/out)
    if(controller.getR1ButtonPressed()){
      algaeControlMotor.setVoltage(3);
    }else if(controller.getL1ButtonPressed()){
      algaeControlMotor.setVoltage(-3);
    }else{
      algaeControlMotor.setVoltage(0);
    }
  }
  //coral control

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

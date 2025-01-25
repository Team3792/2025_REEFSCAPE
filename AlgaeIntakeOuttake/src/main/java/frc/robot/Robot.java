// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  
   //motors
    SparkMax deAlgaefyerMotor = new SparkMax(0, MotorType.kBrushless);

    SparkMax algaeControlMotor = new SparkMax(10, MotorType.kBrushless);
    TalonFX algaeRotateMotor = new TalonFX(22);
    
    SparkMax coralControlMotorRight = new SparkMax(11, MotorType.kBrushless);
    SparkMax coralControlMotorLeft = new SparkMax(12, MotorType.kBrushless);

    TalonFX elevatorMotorLeft = new TalonFX(0);
    TalonFX elevatorMotorRight = new TalonFX(0);

    TalonFX climbMotorRight = new TalonFX(0);
    TalonFX climbMotorLeft = new TalonFX(0);

    Spark led = new Spark(0);
  //controllers
    PS5Controller driver = new PS5Controller(0);
    PS5Controller operator = new PS5Controller(1);

    

  public Robot() {

    TalonFXConfiguration algaeConfiguration = new TalonFXConfiguration();
    algaeConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    algaeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    //Set PID

    //Soft limits
    algaeConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2.2;
    algaeConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    algaeConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    algaeConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;



    //Apply config
    algaeRotateMotor.getConfigurator().apply(algaeConfiguration);
    
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    algaeRotateMotor.setPosition(0); //Zero 
    //algaeRotateMotor.setPosition(0);
    //algaeRotateMotor.set(0);
    
    //algaeRotateMotor.setPosition(100);
  
  }

  @Override
  public void teleopPeriodic() {
    System.out.println("rotate pos"+ algaeRotateMotor.getPosition());

    //driver controls
    //algaeRotateMotor.setVoltage(-1*(driver.getR2Axis() - driver.getL2Axis()));

if(driver.getR2Button()){
     deAlgaefyerMotor.set(0.05);
     led.set(0.79);

    }else if(driver.getL2Button()){
    deAlgaefyerMotor.set(-0.05);
    }else{
      deAlgaefyerMotor.set(0);
    }

    // if(driver.getR1Button()){
    //  algaeControlMotor.set(0.25);
    //  led.set(0.79);

    // }else if(driver.getL1Button()){
    // algaeControlMotor.set(-0.25);
    // }else{
    //   algaeControlMotor.set(0);
    // }

    // if(driver.getR2Button() & algaeRotateMotor.getPosition() ){
      
    // }else{
    //   //algaeRotateMotor.setPosition(100);
    // }
    

    //operator controls
    if(operator.getR1Button()){
      coralControlMotorLeft.set(0.3);
      coralControlMotorRight.set(0.3);
    }else if(operator.getL1Button()){
      coralControlMotorLeft.set(-0.3);
      coralControlMotorRight.set(-0.3);
    }else{
      coralControlMotorLeft.set(0);
      coralControlMotorRight.set(0);
    }

    if(operator.getR2Button()){
      elevatorMotorLeft.set(0.3);
      elevatorMotorRight.set(0.3);
    }else if(operator.getL2Button()){
      elevatorMotorLeft.set(-0.3);
      elevatorMotorRight.set(-0.3);
    }else{
      elevatorMotorLeft.set(0);
      elevatorMotorRight.set(0);
    }

    if(operator.getCrossButton()){
      led.set(0.61);//L1solid red
    }if(operator.getCircleButton()){
      led.set(0.65);//L2,solid orange
    }if(operator.getTriangleButton()){
      led.set(0.69);//L3,solid yellow
    }

    if(operator.getR3Button()){
      climbMotorLeft.set(0.3);
      climbMotorRight.set(0.3);
    }else if(operator.getL3Button()){
      climbMotorLeft.set(-0.3);
      climbMotorRight.set(-0.3);
    }else{
      climbMotorLeft.set(0);
      climbMotorRight.set(0);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Coral.Coral;
import frc.robot.Subsystems.LED.LED;
import frc.robot.Subsystems.LED.LEDConstants;
import frc.robot.HardwareMap;
import frc.robot.Util.CANManager;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFX pivot = new TalonFX(HardwareMap.kAlgaeRotate.id());
  SparkMax drive = new SparkMax(HardwareMap.kAlgaeSpin.id(), MotorType.kBrushless);
  double manualVoltage = 0;

  private ProfiledPIDController pidController = AlgaeIntakeConstants.pivotPIDConfig.getController();

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2cPort);
  public boolean manualMode = false;

  public Trigger hasAlgae = new Trigger(this::hasAlgae);

  
  public AlgaeIntake() {
    //Configure motors
    drive.configure(AlgaeIntakeConstants.getDriveConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.getConfigurator().apply(AlgaeIntakeConstants.getPivotConfig());

    pivot.setPosition(0); //TODO: change to absolute encoder
    pidController.setGoal(0);
    pidController.reset(getAngleDegrees()); //Reset position to current angle to generate profile to return to 0 at start

    CANManager.addConnection(HardwareMap.kAlgaeRotate, pivot);

    
  }

  //Returns true when there is algae in manipulator
  public boolean hasAlgae(){
    return colorSensorV3.getProximity() > AlgaeIntakeConstants.kProximityMin;
  }
  
  public void setDriveVoltage(double voltage){
    drive.setVoltage(voltage);
  }

  //applies voltage to drive motor
  public Command intakeVoltageCommand(double voltage, LED led){
    return Commands.startEnd(
      () -> {
        setDriveVoltage(voltage);
        led.setPattern(LEDConstants.kAlgaeManipulation);
      }, 
      () -> setDriveVoltage(0),
      this, led);
  }

  public Command runRolllerCommand(double voltage){
    return Commands.startEnd(
      () -> {
        setDriveVoltage(voltage);
      },
      () -> setDriveVoltage(0)
    );
  }

  private double getAngleDegrees(){
    return pivot.getPosition().getValueAsDouble()/AlgaeIntakeConstants.kPivotRatio * 360;
  }


  public Command getHoldCommand(){
    return new FunctionalCommand(
      () -> {}, 
      () -> {
        if(hasAlgae()){
          setDriveVoltage(AlgaeIntakeConstants.kHoldingVoltage);
        } else {
          setDriveVoltage(0);
        }
      },
      (b) -> {},
      () -> false, 
      this);
  }

  public Command voltageCommand(double voltage){
    //return Commands.startEnd(() -> pivot.setVoltage(voltage), () -> pivot.setVoltage(0), this);
    return Commands.startEnd(() -> {manualVoltage = voltage;}, () -> {manualVoltage = 0;});
  
  }

  //Stows and stops intake
  public Command stowCommand(){
    return setPositionCommand(AlgaeIntakeConstants.kStowPosition)
          .alongWith(this.runOnce(() -> {setDriveVoltage(-0.25);}));
  }

  public void setPosition(double setPointDegrees){
    pidController.setGoal(setPointDegrees);
  }
  
  public Command setPositionCommand(double position){
    return Commands.runOnce(() -> {setPosition(position);});
  }

  public Command manualModeCommand(LED led, Coral coral){
    return Commands.startEnd(
      () -> {
        manualMode = true; 
        coral.setAngle(90);
        led.setPattern(LEDConstants.kClimbMode);},
      () -> {},//manualMode = false; coral.setAngle(0);}, 
      led, this, coral
    );
  }

  private void runToPosition(){
    double gravityFF = -Math.sin(getAngleDegrees() * Math.PI / 180.0) * AlgaeIntakeConstants.kG;
    double velocityFF = AlgaeIntakeConstants.kVelocityFF * pidController.getSetpoint().velocity;
    double pidOutput = manualMode? 0: pidController.calculate(getAngleDegrees());

    pivot.setVoltage(pidOutput + gravityFF + velocityFF + (manualMode? manualVoltage : 0));
  }
  
  @Override
  public void periodic() {
    runToPosition();
    SmartDashboard.putBoolean("Has Algae", hasAlgae());
  }
}

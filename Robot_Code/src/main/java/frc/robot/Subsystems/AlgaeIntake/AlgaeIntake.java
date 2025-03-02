// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.HardwareMap;
import frc.robot.Subsystems.Coral.CoralConstants;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  TalonFX pivot = new TalonFX(HardwareMap.kAlgaeRotate);
  SparkMax drive = new SparkMax(HardwareMap.kAlgaeSpin, MotorType.kBrushless);

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
  private ProfiledPIDController pidController = AlgaeIntakeConstants.pivotPIDConfig.getController();

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensorV3 = new ColorSensorV3(i2cPort);

  public Trigger hasAlgae = new Trigger(this::hasAlgae);

  
  public AlgaeIntake() {
    //Configure motors
    drive.configure(AlgaeIntakeConstants.getDriveConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivot.getConfigurator().apply(AlgaeIntakeConstants.getPivotConfig());

    pivot.setPosition(0); //TODO: change to absolute encoder
    pidController.setGoal(0);
    pidController.reset(getAngleDegrees()); //Reset position to current angle to generate profile to return to 0 at start
  }

  //Returns true when there is algae in manipulator
  public boolean hasAlgae(){
    return colorSensorV3.getProximity() > AlgaeIntakeConstants.kProximityMin;
  }
  
  public void setDriveVoltage(double voltage){
    drive.setVoltage(voltage);
  }

  //applies voltage to drive motor
  public Command intakeVoltageCommand(double voltage){
    return Commands.startEnd(()->{setDriveVoltage(voltage);}, ()->{setDriveVoltage(0);}, this);
  }

  private double getAngleDegrees(){
    return pivot.getPosition().getValueAsDouble()/AlgaeIntakeConstants.kPivotRatio * 360;
  }


  //Deploys and runs intake until algae is detected
  public Command deployAndIntakeCommand(){
    return setPositionCommand(AlgaeIntakeConstants.kAlgaeIntakePosition)
          .andThen(intakeVoltageCommand(AlgaeIntakeConstants.kIntakeVoltage));
          //.onlyWhile(hasAlgae.negate());
  }

  public Command voltageCommand(double voltage){
    return Commands.startEnd(() -> pivot.setVoltage(voltage), () -> pivot.setVoltage(0), this);
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
    return this.runOnce(()-> {setPosition(position);});
  }

  private void runToPosition(){
    double gravityFF = -Math.sin(getAngleDegrees() * Math.PI / 180.0) * AlgaeIntakeConstants.kG;
    double velocityFF = 0;//CoralConstants.kVelocityFF * pidController.getSetpoint().velocity;
    double pidOutput = pidController.calculate(getAngleDegrees());

    pivot.setVoltage(pidOutput + gravityFF + velocityFF);
    //SmartDashboard.putNumber("Coral/velocity", pivot.getVelocity().getValueAsDouble()/15*360.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae current", drive.getOutputCurrent());
    SmartDashboard.putNumber("Algae speed", drive.getEncoder().getVelocity());
    SmartDashboard.putNumber("Distance", colorSensorV3.getProximity());
    SmartDashboard.putNumber("AlgaePivot", getAngleDegrees());

    //double pidOutput = pidController.calculate(getAngleDegrees());
    //runToPosition();
    //double gFF = getAngleDegrees()
  }
}

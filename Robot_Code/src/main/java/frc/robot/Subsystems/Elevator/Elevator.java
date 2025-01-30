// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX follow = new TalonFX(Constants.HardwareAddresses.elevatorFollowMotorID);
  private TalonFX lead = new TalonFX(Constants.HardwareAddresses.elevatorLeadMotorID);

  public ElevatorState targetState = ElevatorState.Stow;

  private PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);

  public Elevator() {
    
    var slot0Configs = new Slot0Configs();

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = ElevatorConstants.kP;
    config.Slot0.kI = ElevatorConstants.kI;
    config.Slot0.kD = ElevatorConstants.kD;
    config.Slot0.kG = ElevatorConstants.kG;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    lead.getConfigurator().apply(config);

    lead.setControl(new Follower(follow.getDeviceID(), true));
  }

  public enum ElevatorState{
    L1, 
    L2, 
    L3,
    Stow,
    AlgaeLow,
    AlgaeHigh,
    Climb,
    InTransit
  }


  //apply voltage to left motor, right motor will follow; useful for testing
  private void setVoltage(double voltage){
    lead.setVoltage(voltage);
  }

  public Trigger atStateTrigger(ElevatorState state){
    return atPositionTrigger().and(new Trigger(() -> state == targetState));
  }


  public Command setVoltageCommand(double voltage){
    return this.runOnce(()-> {setVoltage(voltage);});
  }
  
  //returns true when carriage is at correct position +/- a given tolerance
  public Trigger atPositionTrigger(double tolerance){
    return new Trigger(
      () -> (Math.abs(lead.getClosedLoopError().getValueAsDouble()) < tolerance 
        && Math.abs(follow.getClosedLoopError().getValueAsDouble()) < tolerance));
  }

  public Trigger atPositionTrigger(){
    return atPositionTrigger(ElevatorConstants.kDefaultTolerance);
  }


  //Set position and target state
  public Command setStateCommand(ElevatorState state){
    return this.runOnce(() -> {
      setPosition(ElevatorConstants.getStateTargetPosition(state));
      targetState = state;
    }
    );
  }

  public void setPosition(double setPoint){
    lead.setControl(positionControl.withPosition(setPoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Subsystems/Elevator/leadMotorPosition", lead.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Subsystems/Elevator/followMotorPosition", follow.getPosition().getValueAsDouble());
  }
}

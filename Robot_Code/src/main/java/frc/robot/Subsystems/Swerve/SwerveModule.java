// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve.ModuleConstants.ModuleConfig;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class SwerveModule {
    TalonFX drive, turn;
    CANcoder encoder;
    VelocityVoltage driveController;
    PIDController turnController;
    String name;

    public SwerveModule(ModuleConfig moduleConfig, String name){
        drive = new TalonFX(moduleConfig.driveID());
        turn = new TalonFX(moduleConfig.turnID());
        encoder = new CANcoder(moduleConfig.encoderID());
        this.name = name;

        //Configure hardware
        drive.getConfigurator().apply(ModuleConstants.getDriveConfig());
        turn.getConfigurator().apply(ModuleConstants.getTurnConfig());
        encoder.getConfigurator().apply(ModuleConstants.getEncoderConfiguration(moduleConfig.encoderOffset()));

        //System controllers
        driveController = new VelocityVoltage(0).withSlot(0);
        turnController = ModuleConstants.kTurnPIDConfig.getController(); 
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        drive.setPosition(0);
        //turn.setPosition(0);
        resetTurnEncoder();
    }

    private void resetTurnEncoder () {
        double position = encoder.getPosition().getValueAsDouble() * ModuleConstants.kTurnRatio;
        turn.setPosition(position);
    }

    public void driveVoltage(double voltage){
        drive.setVoltage(voltage);
        double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), 0);
        turn.setVoltage(turnVoltage);
    }


    private double getDrivePosition(){
        return drive.getPosition().getValueAsDouble() * ModuleConstants.kMetersPerRotation;
    }

    private double getDriveVelocity(){
        return drive.getVelocity().getValueAsDouble() * ModuleConstants.kMetersPerRotation;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getTurnPosition());
    }

    private Rotation2d getTurnPosition(){
        return new Rotation2d(turn.getPosition().getValueAsDouble() * ModuleConstants.kWheelRadiansPerRotation);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getTurnPosition());
    }

    public void setState(SwerveModuleState state){
        System.out.println(state.speedMetersPerSecond);
        if(Math.abs(state.speedMetersPerSecond) > ModuleConstants.kMinSpeed){
            state.optimize(getTurnPosition());

            drive.setControl(driveController.withVelocity(state.speedMetersPerSecond / ModuleConstants.kMetersPerRotation));
            
            double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), state.angle.getRadians());
            turn.setVoltage(turnVoltage);
        } else {
            stop();
        }
        
        
    }

    private void stop(){
        turn.setVoltage(0);
            drive.setVoltage(0);
    }

    public void showEncoderPosition(){
        SmartDashboard.putNumber("Swerve/" + name + "/turn rotations", getTurnPosition().getRotations());
        SmartDashboard.putNumber("Swerve/" + name + "/drive position meters", getDrivePosition());
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Subsystems.Swerve.ModuleConstants.ModuleConfig;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class SwerveModule {
    TalonFX drive, turn;
    CANcoder encoder;
    VelocityVoltage driveController;
    PIDController turnController;

    public SwerveModule(ModuleConfig moduleConfig){
        drive = new TalonFX(moduleConfig.driveID());
        turn = new TalonFX(moduleConfig.turnID());
        encoder = new CANcoder(moduleConfig.encoderID());

        //Configure hardware
        drive.getConfigurator().apply(ModuleConstants.getDriveConfig());
        turn.getConfigurator().apply(ModuleConstants.getTurnConfig());
        encoder.getConfigurator().apply(ModuleConstants.getEncoderConfiguration(moduleConfig.encoderOffset()));

        //Create controller
        driveController = new VelocityVoltage(0).withSlot(0);
        //turnController = new PositionVoltage(0).withSlot(0);

        turnController = ModuleConstants.kTurnPIDConfig.getController(); 
        //PIDController
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        drive.setPosition(0);
        turn.setPosition(0);
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
        state.optimize(getTurnPosition());

        drive.setControl(driveController.withVelocity(state.speedMetersPerSecond / ModuleConstants.kMetersPerRotation));
        
        double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), state.angle.getRadians());
        turn.setVoltage(turnVoltage);
        //turn.setControl(turnController.withPosition(state.angle.getRadians() / ModuleConstants.kWheelRadiansPerRotation));
    }
}

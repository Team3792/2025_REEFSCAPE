// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Dealgaefier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Util.ProfiledPIDConfig;

/** Add your docs here. */
public class DealgaefierConstants {
   
    //Motor configuration for pivot. Literal values are permitted 
    public static TalonFXConfiguration dealgaefierConfig(){

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;

    }
    public static final ProfiledPIDConfig kDealgaefierPID = new ProfiledPIDConfig(0, 0, 0, 0, 0);
    public static final double kDealgaefierEncoderOffset = 0;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ClimbConstants {
    //voltage for running climb (robot goes up)
    public static final double kUpVoltage = 5.0;
    public static final double kDownVoltage = -3.0;
    public static final double kGearRatio = 45.0;
    public static final double kRotationsToDegrees = 360.0/kGearRatio;
    public static final double kForwardLimitDegrees = 185.0;

    public static TalonFXConfiguration getConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kForwardLimitDegrees * kRotationsToDegrees;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }



}

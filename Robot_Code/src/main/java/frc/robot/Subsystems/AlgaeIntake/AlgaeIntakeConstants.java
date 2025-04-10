// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Util.ProfiledPIDConfig;

/** Add your docs here. */
public class AlgaeIntakeConstants {
        //pid+g values for postionVoltage control
        public static final ProfiledPIDConfig pivotPIDConfig = new ProfiledPIDConfig(0.1, 0.1, 0, 720, 1440);
        public static final double kG = 0.3;
        public static final double kVelocityFF = 0.0;

        //positions for manipulator
        public static final double kStowPosition = 0; //start of match position
        public static final double kAlgaeEjectPosition = 0; //score into processor position
        public static final double kAlgaeIntakePosition = 60;//intake from ground position
        public static final double kCoralGroundIntakePosition = 100; //intake from ground coral position
        public static final double kCoralEjectPosition = 20; //coral eject position

        //threshold for sensor to detect algae; less than threshold means theres algae
        public static final double kProximityMin = 250;

        public static final double kPivotRatio = 25.0 * 26.0/15.0;
        
        public static final double kIntakeVoltage = 8;
        public static final double kEjectVoltage = -4;
        public static final double kHoldingVoltage = 1;
        public static final double kManualVoltage = 1.5;


        public static TalonFXConfiguration getPivotConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            return config;
        }  

        public static SparkMaxConfig getDriveConfig(){
            SparkMaxConfig config = new SparkMaxConfig();
            config
                .smartCurrentLimit(60, 30)
                .inverted(true)
                .idleMode(IdleMode.kBrake);

            return config;
        }
}

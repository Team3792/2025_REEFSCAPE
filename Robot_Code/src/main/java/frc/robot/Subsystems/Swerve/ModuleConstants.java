// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.HardwareMap;

/** All constants that relate to physical constants of the modules, including module-specific values and those shared by all modules
 * 
*/
public class ModuleConstants {

    //Conversion factors
    public static final double kMetersPerRotation = 0;
    public static final double kWheelRadiansPerRotation = 0;

    //Module configs
    public static final ModuleConfig kFrontLeftConfig = new ModuleConfig(0, 1, 2, 0);
    public static final ModuleConfig kFrontRightConfig = new ModuleConfig(3, 4, 5, 0);
    public static final ModuleConfig kBackLeftConfig = new ModuleConfig(6, 7, 8, 0);
    public static final ModuleConfig kBackRightConfig = new ModuleConfig(9, 10, 11, 0);

    public static MagnetSensorConfigs getEncoderConfiguration(double offset){
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.MagnetOffset = offset;
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //Can be changed
        return config;
    }

    public static Slot0Configs getDriveConfig(){
        //TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs config = new Slot0Configs();
        config.kS = 0;
        config.kP = 0;
        config.kI = 0;
        config.kD = 0;

        return config;
    }

    //TODO: for turn config, fuse with cancoder and encorporate sensor mechanism wrapping (or do old way)


    
    public record ModuleConfig (int driveID, int turnID, int encoderID, double encoderOffset){}
}

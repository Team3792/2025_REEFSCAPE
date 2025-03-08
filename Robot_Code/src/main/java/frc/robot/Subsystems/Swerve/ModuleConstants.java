// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.PIDConfig;

/** All constants that relate to physical constants of the modules, including module-specific values and those shared by all modules
 * 
*/
public class ModuleConstants {

    //Conversion factors
    public static final double kTurnRatio = 15.4299;
    public static final double kMetersPerRotation = Math.PI/7.36*Units.inchesToMeters(4);
    public static final double kWheelRadiansPerRotation = Math.PI*2/(kTurnRatio);

    public static final double kMinSpeed = 0.1;

    //Turn PID
    public static final PIDConfig kTurnPIDConfig = new PIDConfig(5, 0, 0);

    //Module configs
    public static final ModuleConfig kFrontLeftConfig = new ModuleConfig(0, 1, 2, 0.6298828125);
    public static final ModuleConfig kFrontRightConfig = new ModuleConfig(3, 4, 5, -0.484619140625);
    public static final ModuleConfig kBackLeftConfig = new ModuleConfig(6, 7, 8, -0.24560546875);
    public static final ModuleConfig kBackRightConfig = new ModuleConfig(9, 10, 11, 0.3505859375);

    public static MagnetSensorConfigs getEncoderConfiguration(double offset){
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.MagnetOffset = offset;
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; //Can be changed
        config.AbsoluteSensorDiscontinuityPoint = 1.0;
        return config;
    }

    public static TalonFXConfiguration getDriveConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs slot0Config = config.Slot0;
        slot0Config.kS = 0.17;
        slot0Config.kV = 0.11;
        slot0Config.kP = 0.001;
        slot0Config.kI = 0;
        slot0Config.kD = 0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;



        return config;
    }

    public static TalonFXConfiguration getTurnConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        return config;
    }

    //TODO: for turn config, fuse with cancoder and encorporate sensor mechanism wrapping (or do old way)


    
    public record ModuleConfig (int driveID, int turnID, int encoderID, double encoderOffset){}
}

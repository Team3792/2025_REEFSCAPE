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
import frc.robot.HardwareMap.CANAddress;
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
    public static final ModuleConfig kFrontLeftConfig = new ModuleConfig(
                                                            new CANAddress(0, "front left drive", 6), 
                                                            new CANAddress(1, "front left turn", 9), 
                                                            new CANAddress(2, "front left encoder", 10), 
                                                            0.6298828125),
                                    kFrontRightConfig = new ModuleConfig(
                                                            new CANAddress(3, "front right drive", 17), 
                                                            new CANAddress(4, "front right turn", 16), 
                                                            new CANAddress(5, "front right encoder", 15), 
                                                            -0.484619140625),
                                    kBackLeftConfig = new ModuleConfig(
                                                            new CANAddress(6, "back left drive", 14), 
                                                            new CANAddress(7, "back left turn", 12), 
                                                            new CANAddress(8, "back left encoder", 13), 
                                                            -0.24560546875),
                                    kBackRightConfig = new ModuleConfig(
                                                            new CANAddress(9, "back right drive", 11), 
                                                            new CANAddress(10, "back right turn", 8), 
                                                            new CANAddress(11, "back right encoder", 7), 
                                                            0.3505859375);

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


    
    public static record ModuleConfig (CANAddress driveAddress, CANAddress turnAddress, CANAddress encoderAddress, double encoderOffset){}
}

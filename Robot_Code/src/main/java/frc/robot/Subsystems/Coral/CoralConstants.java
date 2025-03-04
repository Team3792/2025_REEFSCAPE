package frc.robot.Subsystems.Coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Util.ProfiledPIDConfig;

public class CoralConstants {
    //Directions - Motor and encoder should be inverted such that DEPLOYING out the bucket is POSITIVE


    public static final double kStowPosition = 0.0;
    public static final double kIntakePosition = 45.0;
    public static final double kMidPosition = 90.0;
    public static final double kDumpPosition = 120.0;

    //PID control constants - tune gravity and velocity FF first (after max v and a), then tune the PID
    public static final  ProfiledPIDConfig kPivotPID = new ProfiledPIDConfig(0.02, 0, 0, 720, 720);
    public static final double kGravityFF = 0.13;//0.43;
    public static final double kVelocityFF = 0.005;

    public static final double kEncoderOffset = 0.267;

    //Pivot constants
    public static final double kAutoIntakeTime = 1;
    public static final double kAutoDumpTime = 1;


    //Motor configuration for pivot. Literal values are permitted 
    public static TalonFXConfiguration pivotConfig(){

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;

    }
}

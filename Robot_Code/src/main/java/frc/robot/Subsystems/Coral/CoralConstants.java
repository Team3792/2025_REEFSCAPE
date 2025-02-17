package frc.robot.Subsystems.Coral;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Util.PIDConfig;
import frc.robot.Util.ProfiledPIDConfig;

public class CoralConstants {
    //Directions - Motor and encoder should be inverted such that DEPLOYING out the bucket is POSITIVE


    public static final double kStowPosition = 0.0;
    public static final double kIntakePosition = 0.0;
    public static final double kDumpPosition = 0.0;

    //PID control constants - tune gravity and velocity FF first (after max v and a), then tune the PID
    public static final ProfiledPIDConfig kPivotPID = new ProfiledPIDConfig(0, 0, 0, 0, 0);
    public static final double kGravityFF = 0.0;
    public static final double kVelocityFF = 0.0;

    //Pivot constants
    public static final double kAutoIntakeTime = 1;
    public static final double kAutoDumpTime = 1;


    //Motor configuration for pivot. Literal values are permitted 
    public static SparkMaxConfig pivotConfig(){
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(30, 40)
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.closedLoop //May not use if using custom PID
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0)
            .i(0)
            .d(0);
        config.absoluteEncoder
            .zeroOffset(0)
            .inverted(false)
            .positionConversionFactor(360.0);
        //Add more later

        return config;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** All swerve constants that don't relate to the modules*/
public class SwerveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        new Translation2d(kTrackWidth/2, kTrackWidth/2), //Front left
        new Translation2d(kTrackWidth/2, -kTrackWidth/2), //Front right
        new Translation2d(-kTrackWidth/2, kTrackWidth/2), //Back left
        new Translation2d(-kTrackWidth/2, -kTrackWidth/2) //Back right
    );

    //Driving control
    public static final double kMaxSpeedMetersPerSecond = 5;
    //controller values
    public static final double kDeadBandValue = 0.01;

    public static double deadBandClamp(double value){
        if(value < Math.abs(kDeadBandValue)){
            return kDeadBandValue;
        }
        return 0;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.PIDConfig;
import frc.robot.Util.ProfiledPIDConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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

    public static final double kDeadBandValue = 0.05;

    public static final double kMaxLinearVelocity = 4.0; //Left stick
    public static final double kMaxOmega = 4.0; //Right stick 

    public static final double kSlowLinearVelocity = 1.0;
    public static final double kSlowOmega = 1.0;

    //Auto/vision tuning
    public static final PIDConstants kTranslationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);

    public static final PIDConfig kTranslationAlignPIDConfig = new PIDConfig(3.0, 0.0, 0.0);
    public static final ProfiledPIDConfig kRotationAlignPIDConfig = new ProfiledPIDConfig(0.21, 0.0, 0.0, 360, 360);
    public static final TrajectoryConfig kAutoAlignTrajectoryConfig = new TrajectoryConfig(2.0, 2.0);
    public static final PathConstraints kAutoAlignPathConstraints = new PathConstraints(2.0, 2.0, 360, 720);

    public static final Translation2d kCenterAlign = new Translation2d(0.09, -0.18);
    public static final Pose2d kAutoAlignTolerance = new Pose2d(0.005, 0.005, Rotation2d.fromDegrees(2));

    public static final double kRobotTippedDegrees = 10;//how many degreees root needs to be pitched to be considered stuck on algae
}

    

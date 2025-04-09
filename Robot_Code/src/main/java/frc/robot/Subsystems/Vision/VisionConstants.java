// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


/** Add your docs here. */
public class VisionConstants {
    public static final String kCoralCameraName = "coral";
    public static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(14), 0.0, 0.0), //TODO: set these values to be acurate to how the camera is mounted
        new Rotation3d(0, 15.0 * Math.PI/180.0, 0)
        );

    
    public static final int [] reefAprilTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final int [] coralStationAprilTags = {1, 2, 12, 13};
    public static final int [] processorAprilTags = {3, 16};
    
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class VisionConstants {
    public static final String kCoralCameraName = "coral";
    public static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0), //TODO: set these values to be acurate to how the camera is mounted
        new Rotation3d(0, 0, 0)
        );
}

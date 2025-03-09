// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

//TODO: Add compensation for different alliances (Right now, it works for blue alliance)
public class AutoAimGeometry {
    private final Translation3d goal = new Translation3d(1, 5.7, 2.5); //Add measurment later
    private final double centerHeight = 0; //Height off floor for "center" of robot
    private final double relativeHeight = goal.getZ() - centerHeight;

    public double getPitch(Pose2d pose){
        double horizontalDistance = pose.getTranslation().getDistance(goal.toTranslation2d());
        return Math.atan(relativeHeight/horizontalDistance) * 180/Math.PI;
    }

    public double getShotHeading(Pose2d pose){
        //Determine vector from robot to goal
        Translation2d robotToGoal = goal.toTranslation2d().minus(pose.getTranslation());//goal.toTranslation2d().minus(pose.getTranslation());

        return robotToGoal.getAngle().getDegrees();
    }

    public AutoAimParameters getParameters(Pose2d pose){
        return new AutoAimParameters(
            getShotHeading(pose), 
            getPitch(pose)
        );
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;

/** Add your docs here. */
public class FieldGeometry {
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape); //TODO: add separate for welded and non welded
    private static List<AprilTag> tags = fieldLayout.getTags(); //TODO: maybe make this only reef tags?

    
    public static Pose2d getTargetPose(Pose2d robotPose, Pose2d tagRelativePose){
        Pose2d tagPose = getClosestTagPose(robotPose);
        return tagRelativePose.plus(new Transform2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation())); //Map tag relative onto field relative tag pose
    }

    
    private static Pose2d getClosestTagPose(Pose2d pose){ //Rewrite of a method in Pose2d already
        //Start min at the first tag
        double minDistance = getPoseDistance(pose, tags.get(0).pose.toPose2d());
        Pose2d closestTagPose = tags.get(0).pose.toPose2d();
        double distance;
        Pose2d testTagPose;
        
        for(int i = 1; i < tags.size(); i++){
            testTagPose = tags.get(i).pose.toPose2d();
            distance = getPoseDistance(pose, testTagPose);
            if(distance < minDistance){
                minDistance = distance;
                closestTagPose = testTagPose;
            }
        }

        return closestTagPose;
    }

    private static double getPoseDistance(Pose2d pose1, Pose2d pose2){
        return pose1.getTranslation().getDistance(pose2.getTranslation());
    }
}

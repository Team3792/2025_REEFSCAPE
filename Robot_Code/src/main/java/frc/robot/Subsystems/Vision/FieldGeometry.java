// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Subsystems.Swerve.AlignToTagCommand.AlignType;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import java.util.ArrayList;

/** Add your docs here. */
public class FieldGeometry {
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape); //TODO: add separate for welded and non welded
    private static List<AprilTag> tags = fieldLayout.getTags(); //TODO: maybe make this only reef tags?
    private static List<Pose2d> reefTags = getTagGroup(VisionConstants.reefAprilTags);
    private static List<Pose2d> coralStationTags = getTagGroup(VisionConstants.coralStationAprilTags);
    private static List<Pose2d> processorTags = getTagGroup(VisionConstants.processorAprilTags);

    
    public static Pose2d getTargetPose(Pose2d robotPose, AlignType alignType, Pose2d tagRelativePose){
        Pose2d tagPose = getClosestTagPose(robotPose, alignType);
        //return tagRelativePose.plus(new Transform2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation())); //Map tag relative onto field relative tag pose
        return tagPose.plus(new Transform2d(tagRelativePose.getX(), tagRelativePose.getY(), tagRelativePose.getRotation())); //Map tag relative onto field relative tag pose

    }

    public static List<Pose2d> getTagGroup(int groupIndecies[]){
        List<Pose2d> tagGroup = new ArrayList<Pose2d>();
        
        for(int i : groupIndecies){
            tagGroup.add(fieldLayout.getTagPose(i).get().toPose2d());
        }
        return tagGroup;
    }

    private static List<Pose2d> getTagGroup(AlignType alignType){
        switch (alignType){
            case Reef:
                return reefTags;
            case CoralStation:
                return coralStationTags;
            case Processor:
                return processorTags;
            default: 
                return reefTags;
        }
    }

    
    public static Pose2d getClosestTagPose(Pose2d pose, AlignType alignType){ //Rewrite of a method in Pose2d already
        //Start min at the first tag
        List<Pose2d> tagGroup = getTagGroup(alignType);
        double minDistance = getPoseDistance(pose, tagGroup.get(0));
        Pose2d closestTagPose = tagGroup.get(0);
        double distance;
        Pose2d testTagPose;
        
        for(int i = 1; i < tagGroup.size(); i++){
            testTagPose = tagGroup.get(i);
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

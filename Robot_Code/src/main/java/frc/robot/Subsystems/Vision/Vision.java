// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.List;

public class Vision {
  /** Creates a new VisionSubsystem. */
  PhotonCamera coralCamera = new PhotonCamera("low camera");
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  PhotonPoseEstimator fieldPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCamera);


  public Vision() {}

  public Optional<EstimatedRobotPose> getFieldPoseEstimate(){
    List<PhotonPipelineResult> results = coralCamera.getAllUnreadResults();
    return fieldPoseEstimator.update(
        results.get(results.size() - 1) //Update with latest result
        );
  }

  public Optional<Pose2d> getTagToCamera(){
        var result = coralCamera.getLatestResult();

        //Check for targets
        if(!result.hasTargets()){
            return Optional.empty();
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();

        double x = cameraToTarget.getX();
        double y = cameraToTarget.getY();
        Rotation2d theta = cameraToTarget.getRotation().toRotation2d();
        double tagX = -x*theta.getCos() - y*theta.getSin();
        double tagY = -y*theta.getCos() + x*theta.getSin();

        return Optional.of(new Pose2d(tagX, tagY, theta.unaryMinus()));
    }

    public Optional<PhotonTrackedTarget> getBestTargetOptional(){
        var result = coralCamera.getLatestResult();

        //Check for targets
        if(!result.hasTargets()){
            return Optional.empty();
        }
        return Optional.of(result.getBestTarget());
    }
}
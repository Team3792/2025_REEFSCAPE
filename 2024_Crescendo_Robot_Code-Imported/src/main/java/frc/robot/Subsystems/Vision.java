// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision {
  /** Creates a new VisionSubsystem. */
  private PhotonCamera coralCamera = new PhotonCamera("low camera");

  public Vision() {}

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
}

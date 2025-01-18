// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private PhotonCamera coralCamera = new PhotonCamera("low camera");

  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<Transform3d> cameraToTarget = getTagToCamera();
    if(cameraToTarget.isPresent()){
      double xTranslationMeters = cameraToTarget.get().getX(); //In m
      double yTranslationMeters = cameraToTarget.get().getY();
      double thetaDegrees = cameraToTarget.get().getRotation().getZ() * 180.0 / Math.PI; //In 


      SmartDashboard.putNumber("vision/x", xTranslationMeters);
      SmartDashboard.putNumber("vision/y", yTranslationMeters);
      SmartDashboard.putNumber("vision/theta", thetaDegrees);
    }
  }

  public Optional<Transform3d> getTagToCamera(){
        var result = coralCamera.getLatestResult();

        //Check for targets
        if(!result.hasTargets()){
            return Optional.empty();
        }

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
        return Optional.of(cameraToTarget);
    }
}

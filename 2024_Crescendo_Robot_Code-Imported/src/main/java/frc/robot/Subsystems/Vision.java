// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class Vision {
    private PhotonCamera coralCamera = new PhotonCamera("low camera");
    

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

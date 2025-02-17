// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ProfiledPIDConfig {
    double p, i, d, maxV, maxA;

    public ProfiledPIDConfig(double p, double i, double d, double maxV, double maxA){
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxV = maxV;
        this.maxA = maxA;
    }

    public ProfiledPIDController getController(){
        return new ProfiledPIDController(p, i, d, new TrapezoidProfile.Constraints(maxV, maxA));
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class PIDConfig {
    double p, i, d;

    public PIDConfig(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public PIDController getController(){
        return new PIDController(p, i, d);
    }
}

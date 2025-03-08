// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.lang.String;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class PIDTuner {
    String name;
    Tuner pTuner, iTuner, dTuner;
    public PIDTuner(String name, double p, double i, double d){
        this.name = name;
        pTuner = new Tuner(p, name + " P");
        iTuner = new Tuner(i, name + " I");
        dTuner = new Tuner(d, name + " D");
    }

    public PIDController getController(){
        return new PIDController(pTuner.getValue(), iTuner.getValue(), dTuner.getValue());
    }
}

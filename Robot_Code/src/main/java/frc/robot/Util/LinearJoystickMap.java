// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.function.Supplier;

/** Add your docs here. */
public class LinearJoystickMap {
    double maxValue, slowValue, deadBand;
    Supplier<Double> function;
    public LinearJoystickMap(Supplier<Double> function, double slowValue, double maxValue, double deadBand){
        this.function = function;
        this.maxValue = maxValue;
        this.deadBand = deadBand;
    }

    public double calculate(){
        return applyDeadband(function.get()) * maxValue;
    }

    private double applyDeadband(double value){
        if(value < deadBand && value > -deadBand){
            return 0;
        }
        return value;
    }

    public double calculate(DriveMode mode){
        return applyDeadband(function.get()) * ((mode == DriveMode.Fast) ? maxValue : slowValue);
    }

    public enum DriveMode{
        Fast,
        Slow
    }
}

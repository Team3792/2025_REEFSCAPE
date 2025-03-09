// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class MechanismMovement {

    public static boolean mechanismInRange(double[] setPoints, double[] measurements, double[] deadbands){
        for(int i = 0; i < setPoints.length; i++){
            if(!mechanismInRange(setPoints[i], measurements[i], deadbands[i])){ //If this measurement is out of range
                return false;
            }
        }
        return true;
    }

    public static boolean mechanismInRange(double setPoint, double[] measurements, double deadBand){
        for(int i = 0; i < measurements.length; i++){
            if(!mechanismInRange(setPoint, measurements[i], deadBand)){
                return false;
            }
        }
        return true;
    }

    public static boolean mechanismInRange(double setPoint, double measurement, double deadband){
       // System.out.println(Math.abs(measurement - setPoint));
        return Math.abs(measurement - setPoint) < deadband;
    }

    public static boolean mechanismInRange(double measurement, double deadband){
        return Math.abs(measurement) < deadband;
    }

}

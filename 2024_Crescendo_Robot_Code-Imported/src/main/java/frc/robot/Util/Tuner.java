// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


//This class is meant to help tune values for PID and limits
public class Tuner {
    double value;
    String name;
    GenericEntry entry;
    
    public Tuner(double initialValue, String tagName){
        value = initialValue;
        name = tagName;
        firstDisplay();
    }

    public void firstDisplay(){
        //If key isn't already on the smart dashboard, add it
        //This will allow values to be saved between compiling
        // if(!SmartDashboard.containsKey(name)){
        //     SmartDashboard.putNumber(name, value);
        // }
        entry = Shuffleboard.getTab("Outreach Settings").addPersistent(name, value).getEntry();
    }

    public void updateValue(){
        value = entry.get().getDouble();
    }

    public double getValue(){
        updateValue();
        return value;
    }
}
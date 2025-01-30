// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.AlgaeIntake;

/** Add your docs here. */
public class AlgaeIntakeConstants {
    //pid+g values for postionVoltage control
        public static final double kG = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        //positions for manipulator
        public static final double kStowPosition = 0; //start of match position
        public static final double kAlgaeEjectPosition = 0; //score into processor position
        public static final double kAlgaeIntakePosition = 0;//intake from ground position
        //threshold for sensor to detect algae; less than threshold means theres algae
        public static final double kProximityMin = 80.0;
        
        public static final double kIntakeVoltage = 8;
        public static final double kEjectVoltage = -8;
}

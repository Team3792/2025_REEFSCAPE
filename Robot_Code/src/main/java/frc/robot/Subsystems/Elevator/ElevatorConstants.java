// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import frc.robot.Subsystems.Elevator.Elevator.ElevatorState;

/** Add your docs here. */
public class ElevatorConstants {
     //Tuning constants
     public static final double kG = 0;
     public static final double kP = 0;
     public static final double kI = 0;
     public static final double kD = 0;

     //Elevator motor positions for each level of reef interactions
     public static double getStateTargetPosition(ElevatorState state){
        switch(state){
            case L1: return 0;
            case L2: return 0;
            case L3: return 0;
            case Stow: return 0;
            case AlgaeHigh: return 0;
            case AlgaeLow: return 0;
            case Climb: return 0;
            case InTransit: return 0;
            default: return -1;
        }
     }

     public static final double kDefaultTolerance = 0;
}

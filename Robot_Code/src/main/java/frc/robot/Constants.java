// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class HardwareAddresses{ //CAN ID, Rio port, etc.
        //climb
        public static final int climbLeftMotorID = 0;
        public static final int climbRightMotorID = 1;
        
        //coral manipulation
        public static final int coralLeftMotorID = 0;
        public static final int coralRightMotorID = 0;

        public static final int frontSwitchID = 0;
        public static final int backSwitchID = 0;
        //elevator
        public static final int elevatorFollowMotorID = 0;
        public static final int elevatorLeadMotorID = 0;

        //Algae
        public static final int kAlgaeRotateID = 30;
        public static final int kAlgaeSpinID = 31;

        public static final int AlgaeRemoverID = 40;


    }
    public class CoralSubsystem{
        public static final double kswitchVoltageThreshold = 0;
    }
    public class ElevatorSubsystem{
       
    }
    public class AlgaeIntakeSubsystem{
        public static final double kG = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kAlgaeOuttakePosition = 0;
        public static final double kAlgaeIntakePosition = 0;
        public static final double kProximityMin = 80.0;
        public static final double kDeployPosition = 0;
        public static final double kIntakeVoltage = 8;
        public static final double kStowPosition = 0;
        public static final double kEjectVoltage = -8;
    }
    public class ClimbSubsystem{
        public static final double kG = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
    public class LEDSubsystem{
       
    }
}

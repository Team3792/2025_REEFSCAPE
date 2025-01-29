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
        //pid constants
        public static final double kG = 0;//feed forward value
        public static final double kP = 0;//proportional value
        public static final double kI = 0;//integral value
        public static final double kD = 0;//derivative value
        //elevator motor positions for each level of reef interactions
        public static final double kElevatorL1Position = 0;//L1
        public static final double kElevatorL2Position = 0;//L2
        public static final double kElevatorL3Position = 0;//L3
        public static final double kElevatorLevelIntakePosition = 0;//intake coral
        public static final double kElevatorLowerAlgaePosition = 0;//remove lower level algae
        public static final double kElevatorUpperAlgaePosition = 0;//remove upper level algae
        public static final double kElevatorAtPositionParameters = 0.01; //used to set atPosition method true-> if elevator is within 1 cm
        public static final double kDefaultTolerance = 0;
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
        public static final double kIdleLED = 0.99;//(rainbow) robot has no game pieces, and is not moving -> use at start of match?
        public static final double kDriveLED = 0.71; //(lawn green) no game pieces being controlled
        public static final double kElevatorL1LED = 0.61;//(solid red) elevator at level 1 position
        public static final double kEelvatorL2LED = 0.65;//(solid orange) elevator at level 2 position
        public static final double kElevatorL3LED = 0.69;//(solid yellow) elevator at level 3 position
        //public static final double KelevatorL4LED;//not assigned
        public static final double kCoralControlledLED = 0.93; //(white) coral is in mainpulator
        public static final double kReadyToIntakeCoralLED = -0.05; //(strobing white) robot is ligned up to recieve coral from station -> human player indicator
        public static final double kRightAlignedLED = 0.63; //(red orange) // robot is ligned up with right branch on reef
        public static final double kLeftAlignedLED = 0.8;//(solid blue) robot is ligned up with left branch on reef
        public static final double kAlgaeControlledLED = 0.81; //(Aqua) algae is in manipulator
        public static final double kScoringLED = 0.73; //(lime) scoring game piece, coral or algae
        public static final double kClimbLED = 0.57; //(hotpink) robot is climbing, going up or down
    }
}

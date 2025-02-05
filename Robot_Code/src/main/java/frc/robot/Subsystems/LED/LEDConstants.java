// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LED;

/** Add your docs here. */
public class LEDConstants {
    public static final double 
        kIdleLED = 0.99,//(rainbow) robot has no game pieces, and is not moving -> use at start of match?
        kDriveLED = 0.71, //(lawn green) no game pieces being controlled
        kElevatorL1LED = 0.61,//(solid red) elevator at level 1 position
        kEelvatorL2LED = 0.65,//(solid orange) elevator at level 2 position
        kElevatorL3LED = 0.69,//(solid yellow) elevator at level 3 position
        kELevatorAlgaeHighLED = 0.97,//(Dark gray) elevator at high algae remover height
        KElevatorAlgaeLowLED = 0.97,//(Dark Gray) elevator at lower algae remover height
        kCoralControlledLED = 0.93, //(white) coral is in mainpulator
        kReadyToIntakeCoralLED = -0.05, //(strobing white) robot is ligned up to recieve coral from station -> human player indicator
        kRightAlignedLED = 0.63, //(red orange) // robot is ligned up with right branch on reef
        kLeftAlignedLED = 0.8,//(solid blue) robot is ligned up with left branch on reef
        kAlgaeControlledLED = 0.81, //(Aqua) algae is in manipulator
        kScoringLED = 0.73, //(lime) scoring game piece, coral or algae
        kClimbLED = 0.57; //(hotpink) robot is climbing, going up or down
}

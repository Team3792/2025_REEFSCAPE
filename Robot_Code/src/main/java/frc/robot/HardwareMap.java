// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class HardwareMap {
    //Controllers
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final int kPDH = 1; //On the REV CAN bus

    //swerve 0-12 (see ModuleConstants)
    public static final CANAddress kPigeon = new CANAddress(12, "pigeon", 0);

    //Coral
    public static final CANAddress kCoralPivot = new CANAddress(20, "coral pivot", 1);
    public static final int kCoralEncoder = 0;

    //AlgaeIntake
    public static final CANAddress kAlgaeRotate = new CANAddress(30, "algae pivot", 2);
    public static final CANAddress kAlgaeSpin = new CANAddress(31, "algae spin", 3);

    //Climb
    public static final CANAddress kclimbLeft = new CANAddress(50, "climb left", 4);
    public static final CANAddress kclimbRight = new CANAddress(51, "climb right", 5);

    //LED
    public static final int kLED = 60;

    //public static final CANADress 
    public record CANAddress(int id, String name, int index){}
}

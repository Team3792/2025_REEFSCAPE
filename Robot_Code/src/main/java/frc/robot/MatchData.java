// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class MatchData {
    public static final boolean kIsCompetition = true;


    //Returns whether the field should be flipped for red. If data is no available, assume blue.
    public static boolean flipFieldToRed(){
         var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
    }
}

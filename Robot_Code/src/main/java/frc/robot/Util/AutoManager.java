// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MatchData;

/** Add your docs here. */
public class AutoManager {
    public static SendableChooser<Command> chooser = new SendableChooser<>();

    public static void initAutoChooser() {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        List<String> autoNames = AutoBuilder.getAllAutoNames();

        for (String autoName : autoNames) {
            if(!MatchData.kIsCompetition || autoName.startsWith("comp")){ //Check for prefix comp if in competition
                //Create left and right autos
                PathPlannerAuto rightAuto = new PathPlannerAuto(autoName, false);
                PathPlannerAuto leftAuto = new PathPlannerAuto(autoName, true);

                //Add left and right autos with associated prefix
                chooser.addOption(autoName + " Right", rightAuto);
                chooser.addOption(autoName + " Left", leftAuto);
            }
        }
        chooser.setDefaultOption("None", Commands.none());
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public static Command getAuto(){
        return chooser.getSelected();
    }
  }
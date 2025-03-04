// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ConnectionAlert {
    public static void createConnection(String name, BooleanSupplier connectedFunction){
        Trigger connectedTrigger = new Trigger(connectedFunction);
        Alert alert = new Alert(name + " disconnected", AlertType.kError);
        connectedTrigger.onFalse(Commands.runOnce(() -> alert.set(true)));
        connectedTrigger.onTrue(Commands.runOnce(() -> alert.set(false)));
    }
}

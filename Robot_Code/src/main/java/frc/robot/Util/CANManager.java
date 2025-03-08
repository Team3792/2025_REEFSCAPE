// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HardwareMap.CANAddress;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.Alert;

import java.util.ArrayList;

/** Add your docs here. */
public class CANManager {
    private static ArrayList<CANConnection> canConnections = new ArrayList<>();
    public static Alert diconnectionAlert = new Alert("CAN disconnect, see DEBUG", AlertType.kError);

    public static void addConnection(CANAddress address, BooleanSupplier connectedFunction){
        //Sort array for index
        //TODO: fix this sorting, it isn't respected in the dashboard
        for(int i = 0; i < canConnections.size(); i++){
            if(address.index() < canConnections.get(i).address.index()){
                canConnections.add(i, new CANConnection(address, connectedFunction));
                return;
            }
        }
            
        canConnections.add(new CANConnection(address, connectedFunction));
    }

    public static void addConnection(CANAddress address, ParentDevice ctreDevice) {
        addConnection(address, ctreDevice::isConnected);
    }

    public static void updateConnections() {
        boolean allConnected = true;
        for (CANConnection c : canConnections) {
            boolean connected = c.connectedFunction().getAsBoolean();

            SmartDashboard.putBoolean("CANConnections/" + c.address.name() + " [" + c.address.index() +  "] - " + c.address.id(), connected);
            if(!connected){
                allConnected = false;
            }
        }
        diconnectionAlert.set(!allConnected);
    }

    public record CANConnection(CANAddress address, BooleanSupplier connectedFunction) {
    }
}

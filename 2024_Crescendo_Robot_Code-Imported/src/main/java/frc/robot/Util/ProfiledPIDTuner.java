// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class ProfiledPIDTuner {
    ProfiledPIDConfig pidConfig;
    String name;
    Tuner p, i, d, v, a;
    public ProfiledPIDTuner(ProfiledPIDConfig initialConfig, String controllerName){
        pidConfig = initialConfig;
        name = controllerName;
        //Create tuner objects
        p = new Tuner(0, controllerName + "-P");
        i = new Tuner(0, controllerName + "-I");
        d = new Tuner(0, controllerName + "-D");
        v = new Tuner(0, controllerName + "-V");
        a = new Tuner(0, controllerName + "-A");
    }

    public ProfiledPIDConfig getConfig(){
        return new ProfiledPIDConfig(
            p.getValue(),
            i.getValue(),
            d.getValue(),
            v.getValue(),
            a.getValue()    
        );
    }
}

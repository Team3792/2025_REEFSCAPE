/*This class is meant to deal with all signal processing of joystick inputs or other -1 to +1 type inputs  */
//Might add a new parameter to specify maxInput if the input wasn't going to be -1 to +1
//TODO: Test on Shuffle Board

package frc.robot.Util;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SignalProcessor {

    Supplier<Double> input;
    double maxOutput;
    double maxRate;
    int curveType;
    double deadBand;
    SlewRateLimiter slewRateLimiter;
    double lastOutput = 0; //This is used when accesing the output when we don't want to update the slewRateLimiter

    public SignalProcessor(Supplier<Double> input, double maxOutput, double maxRate, int curveType, double deadBand){
        //Set parameters
        this.input = input;
        this.maxOutput = maxOutput;
        this.curveType = curveType;
        this.maxRate = maxRate;
        this.deadBand = deadBand;

        //Create the slew rate limiter. This is symmetric and has an initial value of 1
        slewRateLimiter = new SlewRateLimiter(maxRate);
    }

    public double applyCurve(double val){
        switch(curveType){
            case 0: //Linear
                return val;
            case 1: //Power
                return val*val*val;
            case 2: //Sigmoid
                return (1+Math.exp(-1)) / ((1+Math.exp(-val))-0.5);
        }
        return 0;
    }

    public void setMax(double max){
        maxOutput = max;
    }


    public double calculate(){
        double currentInputWithDeadBand = applyDeadBand(input.get());
        
        double mappedCurvedInput = applyCurve(currentInputWithDeadBand)*maxOutput; 
        double output = mappedCurvedInput;//slewRateLimiter.calculate(mappedCurvedInput);
        //double output = mappedCurvedInput;

        lastOutput = output;

        return output;
    }

    public void resetRateLimiter(){
        slewRateLimiter.reset(0);
    }

    public double applyDeadBand(double val){
        return (Math.abs(val) > deadBand)? val : 0;
    }

    public void displayRawProcessedInput(String inputName){
        SmartDashboard.putNumber("Raw " + inputName, input.get());
        SmartDashboard.putNumber("Processed " + inputName, lastOutput);
    }

}

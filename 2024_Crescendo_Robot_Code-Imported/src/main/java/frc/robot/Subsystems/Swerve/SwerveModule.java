package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

/** Add your docs here. */
public class SwerveModule {
    TalonFX driveMotor, turnMotor;
    DutyCycleEncoder encoder;
    PIDController pidController;
    String moduleName;
    double encoderOffset;
    boolean encoderReversed;
    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset, boolean encoderReversed, String name){
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        encoder = new DutyCycleEncoder(encoderID);
        this.encoderOffset = encoderOffset;
        this.encoderReversed = encoderReversed;
        moduleName = name;
        //driveMotor.config
        System.out.println(encoderReversed);
        
        //pidController = Constants.Swerve.kTurnPIDTuner.getController();
        pidController = new PIDController(0.2, 0, 0);
        pidController.enableContinuousInput(-Math.PI, Math.PI);

        //Configure settings in motors
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration turnConfiguration = new TalonFXConfiguration();
        turnConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnMotor.getConfigurator().apply(turnConfiguration);


        resetEncoders();
    }


    //Reset drive and turn motors to 0
    public void resetEncoders(){
        double absolutePosition = ((encoderReversed)? -1: 1) * Constants.Swerve.kTurnRatio * (encoder.get() - encoderOffset);
        System.out.println(absolutePosition);
        driveMotor.setPosition(0);
        turnMotor.setPosition(absolutePosition);
    }

    public double  getEncoder(){
        //When tuning, use the absolute encoder position
        //return turnMotor.getPosition().getValueAsDouble();
        return encoder.get();
    }

    public double getTurnPositionRadians(){
        return turnMotor.getPosition().getValueAsDouble() * Constants.Swerve.kTurnToRadians;
    }

    public double getDriveVelocity(){
        return driveMotor.getVelocity().getValueAsDouble() * Constants.Swerve.kDriveToMetersPerSecond;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPositionRadians()));
    }

    public double getDriveDistance(){
        return driveMotor.getPosition().getValueAsDouble() * Constants.Swerve.kDriveToMeters;
    }


    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getTurnPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        //If moving very slow, just stop
        if(Math.abs(desiredState.speedMetersPerSecond) < Constants.Swerve.kStopSpeedMetersPerSecond){
            stop();
        }

        //Optimize state - Turn wheel to shortest location to get to desired angle, possibly
        //Switching the rotation of the wheel.
        desiredState.optimize(getState().angle);

        //Apply goals
        double voltage = pidController.calculate(getTurnPositionRadians(), desiredState.angle.getRadians());

        //driveMotor.set(desiredState.speedMetersPerSecond/Constants.Swerve.kMaxSpeedMetersPerSecond);
        driveMotor.setControl(new VelocityVoltage(
            desiredState.speedMetersPerSecond / Constants.Swerve.kDriveToMeters
        ));
        turnMotor.set(
            voltage
        );
    }

    public ChassisSpeeds getSpeeds() {
        return Swerve.kKinematics.toChassisSpeeds(getState());
    }

    // public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
    //     SwerveModuleState[] targetStates = Swerve.kKinematics.toSwerveModuleStates(targetSpeeds);
    //     swerveSubsystem.setModuleState(targetStates);
    // }

    // public SwerveModuleState[] getModuleStates() {
    //     SwerveModuleState[] states = new SwerveModuleState[modules.length];
    //     for (int i = 0; i < modules.length; i++) {
    //       states[i] = modules[i].getState();
    //     }
    //     return states;
    // }

    public void stop(){
        turnMotor.setVoltage(0);
        driveMotor.setVoltage(0);
    }
}

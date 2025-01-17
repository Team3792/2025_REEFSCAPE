//TODO: Add a sprint mode to robot

package frc.robot.Commands.Swerve_Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Util.SignalProcessor;

import java.util.function.Supplier;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveCommand extends Command {
  SwerveSubsystem swerveSubsystem;
  SignalProcessor xOutput, yOutput, tOutput; 
  boolean headingLock = false;
  ProfiledPIDController headingController = Constants.Swerve.headingController.getController();


  public SwerveDriveCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xFunction, Supplier<Double> yFunction, Supplier<Double> tFunction) {
    this.swerveSubsystem = swerveSubsystem;
    headingController.enableContinuousInput(-180, 180);
    xOutput = new SignalProcessor(xFunction, 4, 12, 0, 0.1);
    yOutput = new SignalProcessor(yFunction, 4, 12, 0, 0.1);
    tOutput = new SignalProcessor(tFunction, 4, 12, 0, 0.1);
    headingController.enableContinuousInput(-180, 180);
    addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    headingLock = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(fastMode.get()){
    //   xOutput.setMax(1.5);
    //   yOutput.setMax(1.5);
    // } else if (slowMode.get()){
    //   xOutput.setMax(0.25);
    //   yOutput.setMax(0.25);
    // } else{
    //   xOutput.setMax(0.75);
    //   yOutput.setMax(0.75);
    // }
    double xSpeed = xOutput.calculate();
    double ySpeed = yOutput.calculate();
    double tSpeed = tOutput.calculate();

   


    //Cancel heading lock if tspeed is to high (Still overridden if auto aim it held);
    if(Math.abs(tSpeed) > Constants.Swerve.kMinTurnSpeed){
      headingLock = false;
    }

    //If head locking, recalculate tspeed;
    //SmartDashboard.putNumber("Heading error", headingController.getSetpoint().position);
    if(headingLock){
      tSpeed = headingController.calculate(swerveSubsystem.getRotation2d().getDegrees());
    }
    
    ChassisSpeeds chassisSpeeds = 
     ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed, ySpeed, tSpeed,
      Rotation2d.fromDegrees(swerveSubsystem.getDriverOrientedHeading())
      );

    SwerveModuleState[] moduleStates = Constants.Swerve.kKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleState(moduleStates);
  }

  public void addHeadingLock(double angleRadians){
    headingLock = true;
    headingController.setGoal(angleRadians);
  }

  void showSignals(){
    xOutput.displayRawProcessedInput("x speed");
    yOutput.displayRawProcessedInput("y speed");
    tOutput.displayRawProcessedInput("turn speed");
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

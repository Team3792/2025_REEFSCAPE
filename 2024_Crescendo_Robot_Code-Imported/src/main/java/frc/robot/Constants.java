package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.PIDConfig;
import frc.robot.Util.Tuner;
import frc.robot.Util.ProfiledPIDConfig;

public class Constants {
    public class MotorID{

        //Standard: 0   1
        //          2   3

        //For direction, positive should be its main use (ie. shooting, intaking, climbing)
        public static final int kSwerveFrontLeftDrive = 0;
        public static final int kSwerveFrontLeftTurn = 0;
        public static final int kSwerveFrontRightDrive = 0;
        public static final int kSwerveFrontRightTurn = 0;
        public static final int kSwerveBakcLeftDrive = 0;
        public static final int kSwerveBackLeftTurn = 0;
        public static final int kSwerveBackRightDrive = 0;
        public static final int kSwerveBackRightTurn = 0;
        //intake subsys 10 ~ 19
        public static final int kIntakeMotorTop = 10;
        public static final int kIntakeMotorBottom = 11;
        public static final int kIntakeDeployMotor = 12;
        public static final int kIntakeMotor = 15;
        //shooter subsys 20 ~ 24
        public static final int kLeftFlyWheel = 20;
        public static final int kRightFlyWheel = 21;
        public static final int kPitchMotor = 22;
        //feeder subsys 25 ~ 29
        public static final int kLeftFeeder = 25;
        public static final int kRightFeeder = 26;
        //climber subsys 30 ~ 39
        public static final int kLeftArmMotor = 30;
        public static final int kRightArmMotor = 31;
    }

    public class ButtonID{
        public static final int kIntakeForward = 6;
        public static final int kIntakeBackward = 5;
    }

    public class Shooter{
        public static final double kFlyWheelIntakeSpeed = -1.5;
        public static final double kSubwooferPitchTarget = 70; //it was 65
        public static final Tuner  pitchTuner = new Tuner(kSubwooferPitchTarget, "Front Subwoofer Pitch");
        public static final double kSubwooferBackPitchTarget = 117;//92; //This is a test to distinguish 
        public static final double kAmpPitchTarget = 30;
        public static final double kAmpSpeedTarget = 0;
        public static final double kSubwooferSpeedTarget = 85;//7;
        public static final Tuner subwooferSpeedTargetTuner = new Tuner(kSubwooferSpeedTarget, "Front Subwoofer Speed");
        public static ProfiledPIDConfig kPitchController = new ProfiledPIDConfig(0.05, 0, 0.0, 400, 200);        //public static ProfiledPIDTuner kPitchTuner = new ProfiledPIDTuner(kPitchController, "Pitch Controller");

        public static PIDConfig kFlyWheelController = new PIDConfig(0.01, 0, 0);

        public static final int kPitchEncoderID = 7;
        public static final double kPitchEncoderOffSet = 0;

        public static final double kPitchRatio = 50./36. * 20.;
        public static final int kLightSensor = 3;
        public static final double kMaxPIDCorrection = 1;
        public static final double kZeroVoltage = 2;
        public static final double kStowAngle = -6; //Degrees

        public static final double kReverseShotDebounce = 5; //5 second debounce, change after testing;
    }

    public class Feeder{
        public static final double kIntakeSpeed = 0.5;
        public static final double kFeederSpeed = 01;
    }

    public class Intake{
        public static final double kIntakeVoltage = 8;
        public static final double kOutIntakeSpeed = -8;
        public static final double kDeploymentGoalEncoder = 0;
        public static final double kStowGoalEncoder = 0;
        public static ProfiledPIDConfig kDeployController = new ProfiledPIDConfig(0, 0, 0, 0, 0);
        public static final double kDeployArriveDeadband = 0;
    }

    public class Climber{
        public static final double kMinArmEncoder = -400;
        public static final double kMaxArmEncoder = 0;
        public static ProfiledPIDConfig kArmDeployController = new ProfiledPIDConfig(1, 0, 0, 30, 30);
        public static final double kDeploymentGoalEncoder = -100;
        public static final double kArrivedDeadbandEncoder = 0;
        public static final double kDeployGoal = -400; //Unwind goal in rotations
        public static final double kDeployVoltage = -12;
        public static final double kClimbVoltage = 9;
    }

    public class Swerve{
        //TODO: add conversion ratios
        public static PIDConfig kTurnPIDConfig = new PIDConfig( 0.6, 0, 0);
        public static final double kDriveToMetersPerSecond = 0;
        public static final double kDriveToMeters = Math.PI/6.11*Units.inchesToMeters(4);
        public static final double kTurnRatio = 15.4299;
        public static final double kMinTurnSpeed = 0.1;
        public static final double kTurnToRadians = Math.PI*2/(kTurnRatio);
        public static final double kStopSpeedMetersPerSecond = 0.3;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxSpeedM = 1000;
        public static final double kTranslationMaxSpeedRegular = 10;
        public static final double kTranslationMaxSpeedSprint = 10;
        public static final double kRotationMaxSpeed = 10;
        public static ProfiledPIDConfig headingController = new ProfiledPIDConfig(0.01, 0, 0, 500, 500);
        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        new Translation2d(Units.inchesToMeters(21.5)/2, Units.inchesToMeters(21.5)/2),
        new Translation2d(Units.inchesToMeters(21.5)/2, -Units.inchesToMeters(21.5)/2),
        new Translation2d(-Units.inchesToMeters(21.5)/2, Units.inchesToMeters(21.5)/2),
        new Translation2d(-Units.inchesToMeters(21.5)/2, -Units.inchesToMeters(21.5)/2)
    );
    }

    public class Vision{
        public static final String kCamera1 = "New Camera";
    }

    public class LED{
        public static final int kLEDChannel = 0;

        //Patterns
        public static final double kIdleMode = 0;
        public static final double kIntakeMode = 0;
        public static final double kShootingMode = 0;
    }
}

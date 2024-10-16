package frc.team3128;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.path.PathConstraints;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.Controller.Type;
import common.core.swerve.SwerveConversions;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveEncoderConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import common.utility.shuffleboard.NAR_Shuffleboard;



public class Constants {

    public static class AutoConstants {

        public static final double slowSpeed = 1.5;
        public static final double slowAcceleration = 2;

        public static final PathConstraints constraints = new PathConstraints(
            SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration, SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularAcceleration); 

        /* Translation PID Values */
        public static final double translationKP = 2;
        public static final double translationKI = 0;
        public static final double translationKD = 0;
      
        /* Rotation PID Values */
        public static final double rotationKP = 5;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        public static final double ANGLE_THRESHOLD = 8; //7, 9
        public static final double VELOCITY_THRESHOLD = 4; //6, 3
        public static final double RAMP_THRESHOLD = 9; //8, 10
        public static final double DRIVE_SPEED = Units.inchesToMeters(20); //30, 40

    }

    public static class SwerveConstants {
        public static final double RAMP_TIME = 3;

        public static final int pigeonID = 15; 

        /* Drivetrain Constants */
        public static final double bumperLength = Units.inchesToMeters(5);
        public static final double trackWidth = Units.inchesToMeters(20.75); //Hand measure later
        public static final double wheelBase = Units.inchesToMeters(20.75); //Hand measure later
        public static final double robotLength = Units.inchesToMeters(26.5) + bumperLength; // bumperLength + trackWidth;
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 225.0 / 42.0;
        public static final double angleGearRatio = (300.0 / 13.0); 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); 

        /* Swerve Current Limiting */
        public static final int angleLimit = 30; //30
        public static final int driveLimit = 60; //40;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.15 * 30; // 0.6; // citrus: 0.3 //0.15
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4e-5; //4e-5, //0.05
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.19057;//0.60094; // 0.19225;
        public static final double driveKV = 2.01208;//1.1559;  // 2.4366
        public static final double driveKA = 0.15168; //0.12348; // 0.34415

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double maxSpeed = 4.57;//4.8; //meters per second - 16.3 ft/sec
        public static final double maxAttainableSpeed = maxSpeed; //Stole from citrus.
        public static final double maxAcceleration = 5;
        public static final double maxAngularVelocity = 8; //3; //11.5; // citrus: 10 - Mason look at this later wtf
        public static final double maxAngularAcceleration = 2 * Math.PI; //I stole from citrus.

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final MotorConfig driveMotorConfig = new MotorConfig(SwerveConversions.rotationsToMeters(1, wheelCircumference, driveGearRatio), 60, driveLimit, driveMotorInvert, Neutral.BRAKE);

        public static final MotorConfig angleMotorConfig = new MotorConfig(SwerveConversions.rotationsToDegrees(1, angleGearRatio), 1, angleLimit, angleMotorInvert, Neutral.BRAKE);

        public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(driveKP, driveKI, driveKD, driveKS, driveKV, driveKA);

        public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(angleKP, angleKI, angleKD);

        public static final SwerveModuleConfig Mod0 = new SwerveModuleConfig(
            0, 
            new SwerveMotorConfig(new NAR_TalonFX(1, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(2, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(11, "Drivetrain"), 105.15, canCoderInvert),
            maxSpeed
        );

        public static final SwerveModuleConfig Mod1 = new SwerveModuleConfig(
            1, 
            new SwerveMotorConfig(new NAR_TalonFX(3, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(4, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(12, "Drivetrain"), -62.40234375, canCoderInvert),
            maxSpeed);
        
        public static final SwerveModuleConfig Mod2 = new SwerveModuleConfig(
            2, 
            new SwerveMotorConfig(new NAR_TalonFX(5, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(6, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(13, "Drivetrain"), -84.287109375, canCoderInvert),
            maxSpeed);
        
        public static final SwerveModuleConfig Mod3 = new SwerveModuleConfig(
            3, 
            new SwerveMotorConfig(new NAR_TalonFX(7, "Drivetrain"), driveMotorConfig, drivePIDConfig),
            new SwerveMotorConfig(new NAR_TalonFX(8, "Drivetrain"), angleMotorConfig, anglePIDConfig),
            new SwerveEncoderConfig(new CANcoder(14, "Drivetrain"), 167.607421875, canCoderInvert),
            maxSpeed);

        public static final double turnkP = 5;
        public static final double turnkI = 0;
        public static final double turnkD = 0;
        public static final double turnkS = 0.1; //0.05748
        public static final double turnkV = 0.01723; //0.01723
        public static final double turnkA = 0.0064; //0.0064
        public static final Constraints constraints = new Constraints(Units.radiansToDegrees(maxAngularVelocity), Units.radiansToDegrees(maxAngularAcceleration));
        public static final PIDFFConfig config = new PIDFFConfig(turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, 0);

        public static final Controller TURN_CONTROLLER = new Controller(config, Type.POSITION);
        public static final double TURN_TOLERANCE = 1;

        static {
            TURN_CONTROLLER.enableContinuousInput(-180, 180);
            TURN_CONTROLLER.setMeasurementSource(()-> Swerve.getInstance().getYaw());
            TURN_CONTROLLER.setTolerance(TURN_TOLERANCE);
        }
    }


    public static class VisionConstants {

        public static final double POSE_THRESH = 100;

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(5));

    }
    
    public static class FieldConstants{

        public static final double FIELD_X_LENGTH = Units.inchesToMeters(651.25); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(315.5); // meters
        public static final Pose2d SPEAKER = new Pose2d(Units.inchesToMeters(324.5), Units.inchesToMeters(315.5), Rotation2d.fromDegrees(0));


        public static Pose2d allianceFlip(Pose2d pose) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flip(pose);
            }
            return pose;
        } 

        public static Translation2d allianceFlip(Translation2d translation) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipTranslation(translation);
            }
            return translation;
        }

        public static Rotation2d allianceFlip(Rotation2d rotation) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipRotation(rotation);
            }
            return rotation;
        }

        public static Pose2d flip(Pose2d pose) {
            return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
        }

        public static Translation2d flipTranslation(Translation2d translation) {
            return new Translation2d (
                FIELD_X_LENGTH - translation.getX(),
                translation.getY()
            );
        }

        public static Rotation2d flipRotation(Rotation2d rotation) {
            return Rotation2d.fromDegrees(MathUtil.inputModulus(180 - rotation.getDegrees(), -180, 180));
        }
    }

    public static class FocalAimConstants {
        public static final double speakerLength = 1.043;
        public static final double speakerMidpointY = Units.inchesToMeters(218.29);//5.4;
        
        ; //6.151 - speakerLength / 2;
        public static final double focalPointX = 0.1; //0.229; //1.4583577128;
        public static final Translation2d speakerMidpointBlue = new Translation2d(0, speakerMidpointY);
        public static final Translation2d speakerMidpointRed = new Translation2d(FieldConstants.FIELD_X_LENGTH, speakerMidpointY);
        public static final Translation2d focalPointBlue = new Translation2d(focalPointX, speakerMidpointY);
        public static final Translation2d focalPointRed = new Translation2d(FieldConstants.FIELD_X_LENGTH - focalPointX, speakerMidpointY);
        public static final double angleOffset = 0;
        //testing: kV: drivetrain spinning consistently (ie. v1 = vel at  vel at 1 rad/sec v2=2 rad/sec). 1/(v2-v1) = kV
        //kS: plug kV into 1= kS + kV(v1)
        public static final double offset = 0.3;
        public static final double lowerBound = speakerMidpointY - offset;
        public static final double higherBound = speakerMidpointY + offset;
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID = 60;
        public static final int KICK_MOTOR_ID = 61;
        public static final int KICK_SENSOR_ID = 0;
        public static final int ROLLERS_SENSOR_ID = 1;

        public static final NAR_CANSpark SHOOTER_MOTOR = new NAR_CANSpark(SHOOTER_MOTOR_ID);
        public static final NAR_CANSpark KICK_MOTOR = new NAR_CANSpark(KICK_MOTOR_ID);

        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.0025, 0, 0, 0, 0.00179104, 0); // 0.00187623
        public static final double kF = 0; 
        public static final double GEAR_RATIO = 1;
        public static final double MAX_RPM = 5500;
        public static final double MIN_RPM = 0;
        public static final double TOLERANCE = 150;
        public static final double AMP_RPM = 2500;
        public static final double SHOOTER_RPM = 4500;
        
        public static final double EDGE_FEED_RPM = 5000;
        public static final double EDGE_FEED_ANGLE = 35;
        public static final double MIDDLE_FEED_RPM = 4500;
        public static final double MIDDLE_FEED_ANGLE = 25;
        
        public static final double INTAKE_POWER = 0.65;
        public static final double KICK_POWER = 0.4;
        public static final double KICK_SHOOTING_POWER = 0.8;
        public static final double CURRENT_TEST_POWER = 0;
        public static final double CURRENT_TEST_PLATEAU = 0;
        public static final double CURRENT_TEST_TIMEOUT = 0;
        public static final double CURRENT_TEST_TOLERANCE = 0;
        public static final double CURRENT_TEST_EXPECTED_CURRENT = 0;

        public static final double SHOOTER_TEST_PLATEAU = 1;
        public static final double SHOOTER_TEST_TIMEOUT = 2.5;

        public static final double PROJECTILE_SPEED = 100; // m/s

        public static final double AMP_SHOOTER_POWER = 1;
    }

    public static class IntakeConstants {
        public static final int PIVOT_MOTOR_ID = 31;
        public static final NAR_CANSpark PIVOT_MOTOR = new NAR_CANSpark(PIVOT_MOTOR_ID);

        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.1, 0, 0, -0.35, 0, 0, 0);
        public static final double MAX_VELOCITY = 1000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final PIDFFConfig ROLLER_PID = new PIDFFConfig(0.00218, 0, 0, 0, 0.002, 0);;
        public static final double ROLLER_TOLERANCE = 300;
        public static final double ROLLER_MAX_RPM = 5500;
        public static final double ROLLER_MIN_RPM = 0;

        public static final double ANGLE_TOLERANCE = 3;
        public static final double MIN_SETPOINT = 0;
        public static final double MAX_SETPOINT = 147;
        public static final int CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 1.0 / 40.0;
        public static final double UNIT_CONV_FACTOR = GEAR_RATIO * 360;   

        public static final int ROLLER_MOTOR_ID1 = 31;
        public static final NAR_TalonFX ROLLER_MOTOR1 = new NAR_TalonFX(ROLLER_MOTOR_ID1);
        
        public static final int ROLLER_MOTOR_ID2 = 32; //TODO: ADD
        public static final NAR_TalonFX ROLLER_MOTOR2 = new NAR_TalonFX(ROLLER_MOTOR_ID2);

        public static final double STALL_CURRENT = 50;
        public static final double STALL_POWER = .05;
        public static final double OUTTAKE_POWER = -0.3;
        public static final double INTAKE_POWER = .75;
        public static final double VOLT_COMP = 9;
    }

    public static class LimelightConstants {
        public static final double TX_THRESHOLD = 1;
        public static final double HORIZONTAL_OFFSET_GOAL = 0;
        public static final double PLATEAU_THRESHOLD = 5;
        public static final double TIMEOUT = 1;
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;

        //auto align
        public static final PIDFFConfig config = new PIDFFConfig(KP, KI, KD);
    }

    public static class LedConstants{
        public static final int CANDLE_ID = 52;
        
        public static final int WHITE_VALUE = 0; //leds used don't have a white value
        
        public static final double r_SPEED = 0.75;
        public static final double c_SPEED = 1;
        public static final int STARTING_ID = 8;
        public static final int PIVOT_COUNT = 200; //dunno what this is for
        public static final int PIVOT_FRONT = 40; //change
        public static final int PIVOT_BACK = 50; //change
        public static final int NUM_LED = PIVOT_FRONT - 10;
        public static final int SPARKING = 1;
        public static final double COOLING = 0.3;
        public static final double HOLDING_SPEED = 2;
        public static final double BRIGHTNESS = 1;
        public static final int OFFSET = 5 + 55;

        public static class RainbowAnimation {
            public static final double BRIGHTNESS = 1;
            public static final double SPEED = 1;

        }

        public enum Colors {
            OFF(0,0,0,false),
            ERROR(255, 0, 0, false),
            PIECE(0, 255, 0, false),
            CONFIGURED(0,255,0,false),
            BLUE(48, 122, 171, false),
            RED(171, 48, 97, false),
            PURPLE(255, 0, 255, false),
            GREEN(0, 255, 0, false),
            ORANGE(255, 50, 0, false),
    
            FLAME(0,0,0,true),
            CHARGE(255, 0, 0, true),
            DISCHARGE(0, 0, 0, true),
            AMP(0,0,0,true);
    
            public final int r;
            public final int b;
            public final int g;
            public final boolean animation;
    
            Colors(int r, int g, int b,boolean animation) {
                this.r = r;
                this.g = g;
                this.b = b;
                this.animation = animation;
            }
    
        }
    }
    
    public static class ClimberConstants {
        public static final int CLIMB_MOTOR_ID = 50;
        // public static final NAR_CANSpark CLIMB_MOTOR = new NAR_CANSpark(CLIMB_MOTOR_ID);

        public static final PIDFFConfig PIDConstants = new PIDFFConfig(2, 0, 0, 0.18, 0, 0, 0.3);//240
        public static final double MAX_VELOCTIY = 10000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCTIY, MAX_ACCELERATION);

        public static final double POSITION_TOLERANCE = 0.5;
        public static final double MIN_SETPOINT = 0;
        public static final double MAX_SETPOINT = 30;
        public static final int CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 1.0 / 15.0;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(1.751) * Math.PI;
        public static final double UNIT_CONV_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100;

    }

    public static class HopperConstants {
        public static final int HOPPER_MOTOR_ID = 40;
        public static final NAR_CANSpark HOPPER_MOTOR = new NAR_CANSpark(HOPPER_MOTOR_ID);
        public static final int HOPPER_FRONT_SENSOR_ID = 0;
        public static final int HOPPER_BACK_SENSOR_ID = 2;

        public static final double STALL_CURRENT = 50;
        public static final double HOPPER_INTAKE_POWER = 0.8;
        public static final double HOPPER_OUTTAKE_POWER = -1;
        public static final double STALL_POWER = 0.05;

        public static final double VOLT_COMP = 9;
        public static final int CURRENT_LIMIT = 40;
    }

    public static class AmperConstants {
        public static final int ELEV_MOTOR_ID = 21;
        public static final NAR_CANSpark ELEV_MOTOR = new NAR_CANSpark(ELEV_MOTOR_ID);

        public static final int ROLLER_MOTOR_ID = 20;
        public static final NAR_CANSpark ROLLER_MOTOR = new NAR_CANSpark(ROLLER_MOTOR_ID, ControllerType.CAN_SPARK_FLEX);

        public static final PIDFFConfig ELEVATOR_PID = new PIDFFConfig(0.75, 0, 0, 0.21115, 0.00182, 0.00182, 0.0);
        public static final double MAX_VELOCTIY = 10000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCTIY, MAX_ACCELERATION);

        public static final PIDFFConfig ROLLER_PID = new PIDFFConfig(12/5500, 0, 0, 0, 0.00179104, 0);;
        public static final double ROLLER_TOLERANCE = 100;
        public static final double ROLLER_MAX_RPM = 5500;
        public static final double ROLLER_MIN_RPM = 0;

        public static final double POSITION_TOLERANCE = 0.25;
        public static final double MIN_SETPOINT = 0;
        public static final double MAX_SETPOINT = 21.25;
        public static final int CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 1.0 / (6 + 2/3);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(0.9023) * Math.PI;
        public static final double UNIT_CONV_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100;

        public static final double ROLLER_POWER = 0.9;

        public static final double AMPER_ANGLE = 31.96;
    }

    public static class Flags {
        // public static final BooleanSupplier hopperHasNote = ()-> Hopper.getInstance().hasObjectPresent();
        // public static final BooleanSupplier shooterHasNote = ()-> Shooter.getInstance().hasObjectPresent();
        // public static final BooleanSupplier hasNoNotes = both(not(hopperHasNote), not(shooterHasNote));
        // public static final BooleanSupplier hasOneNote = xor(hopperHasNote, shooterHasNote);
        // public static final BooleanSupplier hasTwoNotes = both(hopperHasNote, shooterHasNote);

        // public static final BooleanSupplier readyForAdvance = both(hopperHasNote, not(shooterHasNote));

        // public static final BooleanSupplier intakeOut = ()-> Intake.pivot.getMeasurement() > 20;
        // public static final BooleanSupplier amperOut = ()-> Amper.elevator.getMeasurement() > 3;

        // public static final BooleanSupplier shooterRunning = ()-> ShooterConstants.SHOOTER_MOTOR.getAppliedOutput() > 0.1;

        // public static final BooleanSupplier amperStalled = ()-> AmperConstants.ROLLER_MOTOR.getStallCurrent() > 40;
        // public static final BooleanSupplier hopperStalled = ()-> HopperConstants.HOPPER_MOTOR.getStallCurrent() > 40;
        // public static final BooleanSupplier intakeStalled = ()-> IntakeConstants.ROLLER_MOTOR1.getStallCurrent() > 40;

        // public static BooleanSupplier not(BooleanSupplier a) {
        //     return ()-> !a.getAsBoolean();
        // }

        // public static BooleanSupplier both(BooleanSupplier a, BooleanSupplier b) {
        //     return ()-> a.getAsBoolean() && b.getAsBoolean();
        // }

        // public static BooleanSupplier xor(BooleanSupplier a, BooleanSupplier b) {
        //     return ()-> a.getAsBoolean() ^ b.getAsBoolean();
        // }
    }
}



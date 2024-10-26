package frc.team3128;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.path.PathConstraints;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.Controller.Type;
import common.core.swerve.SwerveConversions;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveEncoderConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class AutoConstants {

        public static final double slowSpeed = 1.5;
        public static final double slowAcceleration = 2;

        public static final PathConstraints constraints = new PathConstraints(
            SwerveConstants.MAX_DRIVE_SPEED, SwerveConstants.MAX_DRIVE_ACCELERATION, SwerveConstants.MAX_DRIVE_ANGULAR_VELOCITY, SwerveConstants.MAX_DRIVE_ANGULAR_ACCELERATION); 

        /* Translation PID Values */
        public static final double translationKP = 2;
        public static final double translationKI = 0;
        public static final double translationKD = 0;
      
        /* Rotation PID Values */
        public static final double rotationKP = 2;
        public static final double rotationKI = 0;
        public static final double rotationKD = 0;

        public static final double ANGLE_THRESHOLD = 8; //7, 9
        public static final double VELOCITY_THRESHOLD = 4; //6, 3
        public static final double RAMP_THRESHOLD = 9; //8, 10
        public static final double DRIVE_SPEED = Units.inchesToMeters(20); //30, 40

    }

    public static class SwerveConstants {
        public static final double RAMP_TIME = 3;

        /* Drivetrain Constants */
        public static final double DRIVE_BUMPER_LENGTH = Units.inchesToMeters(5);
        public static final double DRIVE_TRACK_WIDTH = Units.inchesToMeters(20.75); //Hand measure later
        public static final double DRIVE_WHEEL_BASE = Units.inchesToMeters(20.75); //Hand measure later
        public static final double ROBOT_LENGTH = Units.inchesToMeters(26.5) + DRIVE_BUMPER_LENGTH; // bumperLength + trackWidth;
        public static final double DRIVE_WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 225.0 / 42.0;
        public static final double DRIVE_ANGLE_GEAR_RATIO = (300.0 / 13.0); 

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(DRIVE_WHEEL_BASE / 2.0, DRIVE_TRACK_WIDTH / 2.0),
                new Translation2d(DRIVE_WHEEL_BASE / 2.0, -DRIVE_TRACK_WIDTH / 2.0),
                new Translation2d(-DRIVE_WHEEL_BASE / 2.0, DRIVE_TRACK_WIDTH / 2.0),
                new Translation2d(-DRIVE_WHEEL_BASE / 2.0, -DRIVE_TRACK_WIDTH / 2.0)); 

        /* Swerve Current Limiting */
        public static final int DRIVE_ANGLE_CURRENT_LIMIT = 30; //30
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60; //40;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double DRIVE_ANGLE_KP = 0.15 * 30; // 0.6; // citrus: 0.3 //0.15
        public static final double DRIVE_ANGLE_KI = 0.0;
        public static final double DRIVE_ANGLE_KD = 0.0; // 12.0; // citrus: 0
        public static final double DRIVE_ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_MOTOR_KP = 4e-5; //4e-5, //0.05
        public static final double DRIVE_MOTOR_KI = 0.0;
        public static final double DRIVE_MOTOR_KD = 0.0;
        public static final double DRIVE_MOTOR_KF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_MOTOR_KS = 0.19057;//0.60094; // 0.19225;
        public static final double DRIVE_MOTOR_KV = 2.01208;//1.1559;  // 2.4366
        public static final double DRIVE_MOTOR_KA = 0.15168; //0.12348; // 0.34415

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double MAX_DRIVE_SPEED = 4.57;//4.8; //meters per second - 16.3 ft/sec
        public static final double MAX_ATTAINABLE_DRIVE_SPEED = MAX_DRIVE_SPEED; //Stole from citrus.
        public static final double MAX_DRIVE_ACCELERATION = 5;
        public static final double MAX_DRIVE_ANGULAR_VELOCITY = 8; //3; //11.5; // citrus: 10 - Mason look at this later wtf
        public static final double MAX_DRIVE_ANGULAR_ACCELERATION = 2 * Math.PI; //I stole from citrus.

        /* Motor and Sensor IDs */
        public static final int SHOOTER_MOTOR_ID = 60;
        public static final int KICK_MOTOR_ID = 61;
        public static final int PIDGEON_ID = 15; 
        public static final String DRIVETRAIN_CANBUS_NAME = "Drivetrain";

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean DRIVE_ANGLE_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean ANGLE_CANCODER_INVERTED = false;

        public static final MotorConfig driveMotorConfig = new MotorConfig(SwerveConversions.rotationsToMeters(1, DRIVE_WHEEL_CIRCUMFERENCE, DRIVE_MOTOR_GEAR_RATIO), 60, DRIVE_MOTOR_CURRENT_LIMIT, DRIVE_MOTOR_INVERTED, Neutral.BRAKE);

        public static final MotorConfig angleMotorConfig = new MotorConfig(SwerveConversions.rotationsToDegrees(1, DRIVE_ANGLE_GEAR_RATIO), 1, DRIVE_ANGLE_CURRENT_LIMIT, DRIVE_ANGLE_INVERTED, Neutral.BRAKE);

        public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(DRIVE_MOTOR_KP, DRIVE_MOTOR_KI, DRIVE_MOTOR_KD, DRIVE_MOTOR_KS, DRIVE_MOTOR_KV, DRIVE_MOTOR_KA);

        public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(DRIVE_ANGLE_KP, DRIVE_ANGLE_KI, DRIVE_ANGLE_KD);

        public static final double DRIVE_TURN_KP = 1;
        public static final double DRIVE_TURN_KI = 0;
        public static final double DRIVE_TURN_KD = 0;
        public static final double DRIVE_TURN_KS = 0.1; //0.05748
        public static final double DRIVE_TURN_KV = 0.01723; //0.01723
        public static final double DRIVE_TURN_KA = 0.0064; //0.0064
        public static final Constraints DRIVE_CONSTRAINTS = new Constraints(Units.radiansToDegrees(MAX_DRIVE_ANGULAR_VELOCITY), Units.radiansToDegrees(MAX_DRIVE_ANGULAR_ACCELERATION));
        public static final PIDFFConfig DRIVE_PIDFF_CONFIG = new PIDFFConfig(DRIVE_TURN_KP, DRIVE_TURN_KI, DRIVE_TURN_KD, DRIVE_TURN_KS, DRIVE_TURN_KV, DRIVE_TURN_KA, 0);

        public static final double TURN_TOLERANCE = 30;
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

        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.005, 0, 0, 0, 0.002, 0); // 0.00187623
        public static final double kF = 0; 
        public static final double GEAR_RATIO = 1;
        public static final double MAX_RPM = 5500;
        public static final double MIN_RPM = 0;
        public static final double TOLERANCE = 150;
        public static final double AMP_RPM = 1500;
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

        public static final PIDFFConfig PIDConstants = new PIDFFConfig(0.1, 0, 0, -0.35, 0, 0, 0);
        public static final double MAX_VELOCITY = 1000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final PIDFFConfig ROLLER_PID = new PIDFFConfig(0.00218, 0, 0, 0, 0.002, 0);
        public static final double ROLLER_TOLERANCE = 300;
        public static final double ROLLER_MAX_RPM = 5500;
        public static final double ROLLER_MIN_RPM = -5500;

        public static final double ANGLE_TOLERANCE = 3;
        public static final double MIN_SETPOINT = 0;
        public static final double MAX_SETPOINT = 147;
        public static final int CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 1.0 / 40.0;
        public static final double UNIT_CONV_FACTOR = GEAR_RATIO * 360;   

        public static final int ROLLER_MOTOR_ID1 = 31;
        public static final int ROLLER_MOTOR_ID2 = 32; //TODO: ADD

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
        public static final int PWM_SERVO_ID = 0;

        public static final PIDFFConfig PIDConstants = new PIDFFConfig(2, 0, 0, 0.18, 0, 0, 0.3);//240
        public static final double MAX_VELOCTIY = 10000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCTIY, MAX_ACCELERATION);

        public static final double POSITION_TOLERANCE = 0.5;
        public static final double MIN_SETPOINT = 0;
        public static final double MAX_SETPOINT = 75;
        public static final int CURRENT_LIMIT = 40;

        public static final double GEAR_RATIO = 1.0 / 15.0;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(1.751) * Math.PI;
        public static final double UNIT_CONV_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100;
    }

    public static class HopperConstants {
        public static final int HOPPER_MOTOR_ID = 40;
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
        public static final int ROLLER_MOTOR_ID = 20;

        public static final PIDFFConfig ELEVATOR_PID = new PIDFFConfig(0.95, 0, 0, 0.21115, 0.00182, 0.00182, 0.0); // kp 0.75
        public static final double MAX_VELOCTIY = 10000000;
        public static final double MAX_ACCELERATION = 100000;
        public static final Constraints TRAP_CONSTRAINTS = new Constraints(MAX_VELOCTIY, MAX_ACCELERATION);

        public static final PIDFFConfig ROLLER_PID = new PIDFFConfig(0.00218, 0, 0, 0, 0.002, 0);
        public static final double ROLLER_TOLERANCE = 500;
        public static final double ROLLER_MAX_RPM = 5500;
        public static final double ROLLER_MIN_RPM = 0;

        public static final double POSITION_TOLERANCE = 0.25;
        public static final double MIN_SETPOINT = 0;
        public static final double MAX_SETPOINT = 30; //21.25
        public static final int CURRENT_LIMIT = 80;

        public static final double GEAR_RATIO = 1.0 / (6 + 2/3);
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(0.9023) * Math.PI;
        public static final double UNIT_CONV_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100;

        public static final double ROLLER_POWER = 0.9;

        public static final double AMPER_ANGLE = 31.96;
    }

    
}



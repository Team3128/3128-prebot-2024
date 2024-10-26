package frc.team3128.subsystems;

import static frc.team3128.Constants.VisionConstants.SVR_STATE_STD;
import static frc.team3128.Constants.VisionConstants.SVR_VISION_MEASUREMENT_STD;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveEncoderConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.commands.CmdSwerveDrive;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private Pigeon2 gyro;

    public double throttle = 1;

    private static double gyroOffset = 0;

    public Supplier<Double> yaw;

    private static final SwerveModuleConfig Mod0 = new SwerveModuleConfig(
        0, 
        new SwerveMotorConfig(new NAR_TalonFX(1, DRIVETRAIN_CANBUS_NAME), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(2, "Drivetrain"), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(11, "Drivetrain"), 105.15, ANGLE_CANCODER_INVERTED),
        SwerveConstants.MAX_DRIVE_SPEED);

    private static final SwerveModuleConfig Mod1 = new SwerveModuleConfig(
        1, 
        new SwerveMotorConfig(new NAR_TalonFX(3, "Drivetrain"), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(4, "Drivetrain"), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(12, "Drivetrain"), -62.40234375, ANGLE_CANCODER_INVERTED),
        SwerveConstants.MAX_DRIVE_SPEED);
        
    private static final SwerveModuleConfig Mod2 = new SwerveModuleConfig(
        2, 
        new SwerveMotorConfig(new NAR_TalonFX(5, "Drivetrain"), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(6, "Drivetrain"), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(13, "Drivetrain"), -84.287109375, ANGLE_CANCODER_INVERTED),
        SwerveConstants.MAX_DRIVE_SPEED);
        
    private static final SwerveModuleConfig Mod3 = new SwerveModuleConfig(
        3, 
        new SwerveMotorConfig(new NAR_TalonFX(7, "Drivetrain"), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(8, "Drivetrain"), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(14, "Drivetrain"), 167.607421875, ANGLE_CANCODER_INVERTED),
        SwerveConstants.MAX_DRIVE_SPEED);

    private final Controller turnController = new Controller(DRIVE_PIDFF_CONFIG, Type.POSITION);
    private double turnSetpoint;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        chassisVelocityCorrection = false;
        Timer.delay(1);
        gyro = new Pigeon2(PIDGEON_ID, "Drivetrain");
        Timer.delay(1);
        var x = gyro.getYaw();
        x.setUpdateFrequency(100);
        yaw = gyro.getYaw().asSupplier();

        gyro.optimizeBusUtilization();

        turnController.enableContinuousInput(-180, 180);
        turnController.setMeasurementSource(()-> Swerve.getInstance().getYaw());
        turnController.setTolerance(TURN_TOLERANCE);

        initShuffleboard();
        NAR_Shuffleboard.addData("Testing", "Name", ()-> getDist(speakerMidpointBlue), 0, 0);
        NAR_Shuffleboard.addData("Testing", "Dist", ()-> getDistHorizontal(), 0, 1);
        NAR_Shuffleboard.addData("Auto", "Setpoint", ()-> turnController.atSetpoint());
        initStateCheck();
    }

    public boolean crossedPodium() {
        final double x = getPose().getX();
        if (Robot.getAlliance() == Alliance.Red) return x > FieldConstants.FIELD_X_LENGTH - 2.1;
        return x < 2.1;
    }

    public void setVoltage(double volts) {
        for (final SwerveModule module : modules) {
            module.getAngleMotor().set(0, Control.Position);
            module.getDriveMotor().setVolts(volts);
        }
    }

    public double getVelocity() {
        var x = getRobotVelocity();
        return Math.hypot(x.vxMetersPerSecond, x.vyMetersPerSecond);
    }

    @Override
    public double getYaw() {
        return yaw.get() - gyroOffset;
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    @Override
    public void zeroGyro(double reset) {
        gyro.setYaw(Robot.getAlliance() == Alliance.Red ? 0 : 180);
        // gyroOffset = (Robot.getAlliance() == Alliance.Red ? 180 : 0) - gyro.getAngle();
    }

    public double getPredictedDistance() {
        final ChassisSpeeds velocity = getFieldVelocity();
        final Translation2d predictedPos = getPredictedPosition(velocity, RAMP_TIME);
        final double shotTime = getProjectileTime(getDist(predictedPos));
        final Translation2d target = calculateTarget(Robot.getAlliance() == Alliance.Red ? speakerMidpointRed : speakerMidpointBlue, velocity, shotTime);
        final double distance = getDist(predictedPos, target);
        return distance;
    }

    public double getPredictedAngle() {
        final ChassisSpeeds velocity = getFieldVelocity();
        final Translation2d predictedPos = getPredictedPosition(velocity, RAMP_TIME);
        final double shotTime = getProjectileTime(getDist(predictedPos));
        final Translation2d target = calculateTarget(Robot.getAlliance() == Alliance.Red ? speakerMidpointRed : speakerMidpointBlue, velocity, shotTime);
        final double angle = getTurnAngle(predictedPos, target);
        return angle;
    }

    public Translation2d getPredictedPosition(ChassisSpeeds velocity, double time) {
        final Translation2d currentPosition = getPose().getTranslation();
        return currentPosition.plus(new Translation2d(velocity.vxMetersPerSecond * time, velocity.vyMetersPerSecond * time));
    }

    public double getProjectileTime(double distance) {
        return distance / ShooterConstants.PROJECTILE_SPEED;
    }

    public Translation2d calculateTarget(Translation2d target, ChassisSpeeds velocity, double time) {
        return target.minus(new Translation2d(0, velocity.vyMetersPerSecond * time));
    }

    public double getDistHorizontal() {
        final double x = getPose().getX();
        final double dist = Robot.getAlliance() == Alliance.Red ? FieldConstants.FIELD_X_LENGTH - x : x;
        return dist - ROBOT_LENGTH / 2.0;
    }

    public double getDist() {
        return getDist(Robot.getAlliance() == Alliance.Red ? speakerMidpointRed : speakerMidpointBlue);
    }

    public double getDist(Translation2d point) {
        return getDist(getPose().getTranslation(), point);
    }

    public double getDist(Translation2d point1, Translation2d point2) {
        return point1.getDistance(point2) - ROBOT_LENGTH / 2.0;
    }

    public double getTurnAngle() {
        return getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue);
    }

    public double getTurnAngle(Translation2d target) {
        final Translation2d robotPos = Swerve.getInstance().getPose().getTranslation();
        return getTurnAngle(robotPos, target);
    }

    public double getTurnAngle(Translation2d robotPos, Translation2d targetPos) {
        return Math.toDegrees(Math.atan2(targetPos.getY() - robotPos.getY(), targetPos.getX() - robotPos.getX())) + angleOffset;
    }

    public Command turnInPlace(boolean moving) {
        return turnInPlace(()-> moving ? getPredictedAngle() : getTurnAngle());
    }

    public Command turnInPlace(DoubleSupplier setpoint) {
        return new NAR_PIDCommand(
            turnController, 
            ()-> getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                final double x = RobotContainer.controller.getLeftX();
                final double y = RobotContainer.controller.getLeftY();
                Translation2d translation = new Translation2d(x,y).times(MAX_ATTAINABLE_DRIVE_SPEED);
                if (Robot.getAlliance() == Alliance.Red || !fieldRelative) {
                    translation = translation.rotateBy(Rotation2d.fromDegrees(90));
                }
                else {
                    translation = translation.rotateBy(Rotation2d.fromDegrees(-90));
                }

                Swerve.getInstance().drive(translation, Units.degreesToRadians(output), true);
            },
            2,
            Swerve.getInstance()
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.setTurnEnabled(false)));
    }

    public boolean isConfigured() {
        for (final SwerveModule module : modules) {
            final double CANCoderAngle = module.getAbsoluteAngle().getDegrees();
            final double AngleMotorAngle = module.getAngleMotor().getPosition();
            if (CANCoderAngle == 0 || AngleMotorAngle == 0) return false;
        }
        return true;
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    public double getTurnSetpoint() {
        return turnSetpoint;
    }

    public void setTurnSetpoint(double turnSetpoint) {
        this.turnSetpoint = turnSetpoint;
        CmdSwerveDrive.setTurnEnabled(true);
    }

    public boolean isTurnControllerAtSetpoint() {
        return turnController.atSetpoint();
    }

    public double getTurnControllerCalculation(double setpoint) {
        return turnController.calculate(getGyroRotation2d().getDegrees(), setpoint);
    }

    public void resetTurnController() {
        turnController.reset();
    }

    @Override
    public void initShuffleboard(){
        super.initShuffleboard();
        NAR_Shuffleboard.addSendable("Commands", "Swerve Commands", this, 0, 0);
    }

}
    


package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.controllers.ControllerBase;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends ShooterTemplate {

    private static Shooter instance;

    public static synchronized Shooter getInstance(){
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    private boolean isShooting;

    private DigitalInput rollersSensor;

    private Shooter() {
        super(new Controller(PIDConstants, Type.VELOCITY), SHOOTER_MOTOR);
        setConstraints(MIN_RPM, MAX_RPM);
        setTolerance(TOLERANCE);

        rollersSensor = new DigitalInput(ROLLERS_SENSOR_ID);
        isShooting = false;

        configMotors();
        // initShuffleboard();
        controller.setTolerance(TOLERANCE);
    }

    @Override
    protected void configMotors() {
        SHOOTER_MOTOR.setCurrentLimit(80);
        KICK_MOTOR.setCurrentLimit(80);
        
        SHOOTER_MOTOR.setInverted(false);
        KICK_MOTOR.setInverted(false);

        SHOOTER_MOTOR.setUnitConversionFactor(GEAR_RATIO);

        SHOOTER_MOTOR.setNeutralMode(Neutral.COAST);
        KICK_MOTOR.setNeutralMode(Neutral.COAST);

        SHOOTER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        KICK_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    public Command rampUpShooter(){
        return sequence(
            runKickMotor(-0.1),
            waitSeconds(0.1),
            runKickMotor(0),
            runShooter(SHOOTER_RPM)
        );
    } 

    public Command runShooter(double power) {
        return runOnce(()-> SHOOTER_MOTOR.set(power));
    }

    public Command runKickMotor(double power) {
        return runOnce(() -> KICK_MOTOR.set(power));
    }

    public Command stopMotors(){
        return runOnce(()-> {
            SHOOTER_MOTOR.set(0);
            KICK_MOTOR.set(0);
        });
    }

    public boolean hasObjectPresent() {
        return !rollersSensor.get();
    }

    public boolean getShooting() {
        return isShooting;
    }

    public Command setShooting(boolean value) {
        return runOnce(() -> isShooting = value);
    }
}

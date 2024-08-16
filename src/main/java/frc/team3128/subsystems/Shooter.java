package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.controllers.ControllerBase;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends ShooterTemplate {

    private static Shooter instance;

    private DoubleSupplier rpmDiff;

    private DoubleSupplier kF_Func;

    private DigitalInput sensor;

    private Shooter() {
        super(new Controller(PIDConstants, Type.VELOCITY));
        setConstraints(MIN_RPM, MAX_RPM);
        setTolerance(TOLERANCE);
        sensor = new DigitalInput(SHOOTER_SENSOR_ID);
        configMotors();
        initShuffleboard();

        ControllerBase controller = getController();
        controller.setkS(()-> controller.getkS());
        controller.setkV(()-> controller.getkV());
        controller.setkG(()-> controller.getkG());
        controller.setTolerance(TOLERANCE);
    }

    public static synchronized Shooter getInstance(){
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    @Override
    protected void configMotors() {
        SHOOTER_MOTOR.setCurrentLimit(80);
        KICK_MOTOR.setCurrentLimit(80);
        
        SHOOTER_MOTOR.setInverted(false);
        KICK_MOTOR.setInverted(false);

        SHOOTER_MOTOR.setUnitConversionFactor(GEAR_RATIO);

        SHOOTER_MOTOR.setNeutralMode(Neutral.COAST);
        KICK_MOTOR.setNeutralMode(Neutral.BRAKE);

        SHOOTER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        KICK_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    @Override
    public void startPID(double setpoint) {
        enable();
        getController().setSetpoint(setpoint);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        SHOOTER_MOTOR.setVolts(setpoint != 0 ? output + kF_Func.getAsDouble() : 0);
    }

    @Override
    public double getMeasurement() {
        return SHOOTER_MOTOR.getVelocity();
    }

    public Command shoot(double setpoint) {
        return runOnce(()-> startPID(setpoint));
    }

    public Command runShooter(double power) {
        return runOnce(()-> setPower(power));
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

    public State getRunningState() {
        if (SHOOTER_MOTOR.getState() != State.DISCONNECTED && KICK_MOTOR.getState() != State.DISCONNECTED)
            return State.RUNNING;
        if (SHOOTER_MOTOR.getState() != State.DISCONNECTED || KICK_MOTOR.getState() != State.DISCONNECTED)
            return State.PARTIALLY_RUNNING;
        return State.DISCONNECTED;
    }

    public boolean hasObjectPresent() {
        return !sensor.get();
    }
}

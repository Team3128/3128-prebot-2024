package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.controllers.ControllerBase;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends ShooterTemplate {

    private static Shooter instance;

    private DoubleSupplier rpmDiff;

    private DoubleSupplier kF_Func;

    private Controller rightController = new Controller(PIDConstants, Type.VELOCITY);

    private Shooter() {
        super(new Controller(PIDConstants, Type.VELOCITY));
        setConstraints(MIN_RPM, MAX_RPM);
        setTolerance(TOLERANCE);
        configMotors();
        initShuffleboard();

        ControllerBase controller = getController();
        rightController.setkS(()-> controller.getkS());
        rightController.setkV(()-> controller.getkV());
        rightController.setkG(()-> controller.getkG());
        rightController.setTolerance(TOLERANCE);
    }

    public static synchronized Shooter getInstance(){
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    @Override
    protected void configMotors() {
        LEFT_MOTOR.setCurrentLimit(80);
        RIGHT_MOTOR.setCurrentLimit(80);
        
        LEFT_MOTOR.setInverted(true);
        RIGHT_MOTOR.setInverted(false);

        LEFT_MOTOR.setUnitConversionFactor(GEAR_RATIO);
        RIGHT_MOTOR.setUnitConversionFactor(GEAR_RATIO);

        LEFT_MOTOR.setNeutralMode(Neutral.COAST);
        RIGHT_MOTOR.setNeutralMode(Neutral.COAST);

        LEFT_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        RIGHT_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    public void startPID(double leftSetpoint, double rightSetpoint) {
        enable();
        getController().setSetpoint(leftSetpoint);
        rightController.setSetpoint(rightSetpoint);
    }

    @Override
    public void startPID(double setpoint) {
        final double rpm = (debug != null && debug.getAsBoolean()) ? this.setpoint.getAsDouble() : setpoint;
        startPID(rpm, rpm - rpmDiff.getAsDouble());
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        LEFT_MOTOR.setVolts(setpoint != 0 ? output + kF_Func.getAsDouble() : 0);
        final double rightOutput = rightController.calculate(Math.abs(RIGHT_MOTOR.getVelocity()));
        RIGHT_MOTOR.setVolts(setpoint != 0 ? rightOutput + kF_Func.getAsDouble() : 0);
    }

    @Override
    public double getMeasurement() {
        return LEFT_MOTOR.getVelocity();
    }

    public Command shoot(double leftSetpoint, double rightSetpoint) {
        return runOnce(()-> startPID(leftSetpoint, rightSetpoint));
    }

    public Command shoot(double setpoint){
        return runOnce(() -> startPID(setpoint));
    }

    public Command setShooter(double power) {
        return runOnce(()-> setPower(power));
    }

    public Command runBottomRollers(double power) {
        return runOnce(()-> RIGHT_MOTOR.set(power));
    }

    public State getRunningState() {
        if (RIGHT_MOTOR.getState() != State.DISCONNECTED && LEFT_MOTOR.getState() != State.DISCONNECTED)
            return State.RUNNING;
        if (RIGHT_MOTOR.getState() != State.DISCONNECTED || RIGHT_MOTOR.getState() != State.DISCONNECTED)
            return State.PARTIALLY_RUNNING;
        return State.DISCONNECTED;
    }
}

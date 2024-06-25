package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.NAR_PIDSubsystem;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.*;
import static frc.team3128.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class Climber extends NAR_PIDSubsystem {
    private static Climber instance;

    private NAR_CANSpark climbMotor;

    private Climber() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS));
        configMotors();
        setTolerance(POSITION_TOLERANCE);
        setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
    }

    public static synchronized Climber getInstance() {
        if (instance == null)
            instance = new Climber();
        return instance;
    }

    private void configMotors() {
        climbMotor = new NAR_CANSpark(CLIMB_MOTOR_ID);
        climbMotor.setInverted(false);
        climbMotor.setUnitConversionFactor(GEAR_RATIO * WHEEL_CIRCUMFERENCE * 100);
        climbMotor.setNeutralMode(Neutral.BRAKE);
        climbMotor.setStatusFrames(SparkMaxConfig.POSITION);
    }

    private void setPower(double power) {
        disable();
        climbMotor.set(power);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        climbMotor.setVolts(output);
    }

    @Override
    protected double getMeasurement() {
        return climbMotor.getPosition();
    }

    public Command climbTo(DoubleSupplier setpoint) {
        return runOnce(()->startPID(setpoint.getAsDouble()));
    }

    public Command climbTo(double setpoint) {
        return climbTo(()->setpoint);
    }

    public State getRunningState() {
        return climbMotor.getState();
    }
}

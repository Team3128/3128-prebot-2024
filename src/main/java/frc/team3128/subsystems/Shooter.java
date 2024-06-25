package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.controllers.ControllerBase;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.tester.Tester;
import common.utility.tester.Tester.UnitTest;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends ShooterTemplate {
    private NAR_CANSpark leftMotor;
    private NAR_CANSpark rightMotor;

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
        leftMotor = new NAR_CANSpark(LEFT_MOTOR_ID, ControllerType.CAN_SPARK_FLEX);
        rightMotor = new NAR_CANSpark(RIGHT_MOTOR_ID, ControllerType.CAN_SPARK_FLEX);

        leftMotor.setCurrentLimit(80);
        rightMotor.setCurrentLimit(80);
        
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setUnitConversionFactor(GEAR_RATIO);
        rightMotor.setUnitConversionFactor(GEAR_RATIO);

        leftMotor.setNeutralMode(Neutral.COAST);
        rightMotor.setNeutralMode(Neutral.COAST);

        leftMotor.setStatusFrames(SparkMaxConfig.VELOCITY);
        rightMotor.setStatusFrames(SparkMaxConfig.VELOCITY);
    }
    @Override 
    public void initShuffleboard() {
        super.initShuffleboard();
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
        leftMotor.setVolts(setpoint != 0 ? output + kF_Func.getAsDouble() : 0);
        final double rightOutput = rightController.calculate(Math.abs(rightMotor.getVelocity()));
        rightMotor.setVolts(setpoint != 0 ? rightOutput + kF_Func.getAsDouble() : 0);
    }

    @Override
    public double getMeasurement() {
        return leftMotor.getVelocity();
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
        return runOnce(()-> rightMotor.set(power));
    }

    public State getRunningState() {
        if (rightMotor.getState() != State.DISCONNECTED && leftMotor.getState() != State.DISCONNECTED)
            return State.RUNNING;
        if (rightMotor.getState() != State.DISCONNECTED || rightMotor.getState() != State.DISCONNECTED)
            return State.PARTIALLY_RUNNING;
        return State.DISCONNECTED;
    }

    public UnitTest getShooterTest() {
        return new SetpointTest
        (
            "testShooter",
            MAX_RPM,
            SHOOTER_TEST_PLATEAU,
            SHOOTER_TEST_TIMEOUT
        );
    }

    public void addShooterTests() {
        Tester.getInstance().addTest("Shooter", getShooterTest());
    }
}

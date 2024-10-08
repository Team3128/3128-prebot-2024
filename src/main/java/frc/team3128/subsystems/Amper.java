package frc.team3128.subsystems;

import static frc.team3128.Constants.AmperConstants.*;
import static frc.team3128.Constants.Flags.*;

import common.core.subsystems.ElevatorTemplate;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.subsystems.Shooter.ShooterState;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Amper extends ElevatorTemplate {
    
    public enum AmpState {
        //TODO: correct EXTEND setpoints
        AMP(POSITION_MAX, ROLLER_POWER),
        PRIMED(POSITION_MAX * 0.8, ROLLER_POWER),
        RETRACTED(0, 0);

        private final double setpoint;
        private final double rollerPower;

        private AmpState(double setpoint, double rollerPower) {
            this.setpoint = setpoint;
            this.rollerPower = rollerPower;
        }

        public double getSetpoint() {
            return setpoint;
        }

        public double getRollerPower() {
            return rollerPower;
        }
    }

    private static Amper instance;
    private static AmpState goalState;

    public static synchronized Amper getInstance() {
        if (instance == null)
            instance = new Amper();
        return instance;
    }

    private Amper() {
        super(CONTROLLER, ELEV_MOTOR);
        configController();
        initShuffleboard();

        setState(AmpState.RETRACTED).schedule();
    }

    @Override
    protected void configMotors() {
        ELEV_MOTOR.setInverted(ELEV_MOTOR_INVERT);
        ELEV_MOTOR.setCurrentLimit(ELEV_CURRENT_LIMIT);
        ELEV_MOTOR.setUnitConversionFactor(ELEV_GEAR_RATIO);
        ELEV_MOTOR.setStatusFrames(ELEV_STATUS_FRAME);
        ELEV_MOTOR.setNeutralMode(ELEV_NEUTRAL_MODE);

        ROLLER_MOTOR.setInverted(ROLLER_MOTOR_INVERT);
        ROLLER_MOTOR.setNeutralMode(ROLLER_NEUTRAL_MODE);
        ROLLER_MOTOR.setUnitConversionFactor(ROLLER_GEAR_RATIO);
        ROLLER_MOTOR.setStatusFrames(ROLLER_STATUS_FRAME);
        ROLLER_MOTOR.setNeutralMode(ROLLER_NEUTRAL_MODE);
    }

    public void configController(){
        CONTROLLER.setConstraints(POSITION_MIN, POSITION_MAX);
        CONTROLLER.setTolerance(POSITION_TOLERANCE);
        this.setSafetyThresh(2);
        // setkG_Function(() ->  getMeasurement()*Math.sin(AMPER_ANGLE));
    }

    public void configTriggers(){
        // assuming that we always want to amp two notes when given the chance
        new Trigger(hasNoNotes)
        .debounce(0.25) // account for hopper to shooter transition
        .onTrue(setState(AmpState.RETRACTED));

        new Trigger(()-> Shooter.goalStateIs(ShooterState.SHOOT))
        .onTrue(setState(AmpState.RETRACTED));
    }
    
    public void setVoltage(double volts) {
        ELEV_MOTOR.set(0, Control.Position);
        ELEV_MOTOR.setVolts(volts);
    }

    public double getVelocity() {
        return ELEV_MOTOR.getVelocity();
    }
    
    public double getPosition() {
        return ELEV_MOTOR.getPosition();
    }

    @Override
    public void initShuffleboard(){
        super.initShuffleboard();

        NAR_Shuffleboard.addData(getName(), "Roller Current", ()-> ROLLER_MOTOR.getStallCurrent(), 5, 2);
    }

    public Command runRollers(double power) {
        return runOnce(() -> ROLLER_MOTOR.set(power));
    }

    public boolean rollerHasStaller(){
        return ROLLER_MOTOR.getStallCurrent() > ROLLER_STALL_THRESHOLD;
    }

    public Command setState(AmpState state) {
        goalState = state;
        return sequence(
            moveElevator(state.getSetpoint()),
            runRollers(state.getRollerPower())
        );
    }

    public static AmpState getGoalState() {
        return goalState;
    }

    public static boolean goalStateIs(AmpState state) {
        return goalState == state;
    }
}
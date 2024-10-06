package frc.team3128.subsystems;

import static frc.team3128.Constants.AmperConstants.POSITION_MAX;
import static frc.team3128.Constants.IntakeConstants.*;

import common.core.subsystems.PivotTemplate;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends PivotTemplate{

    public enum IntakeState {
        INTAKE(POSITION_MAX, INTAKE_POWER),
        OUTTAKE(POSITION_MAX, OUTTAKE_POWER), // lmao dont need
        SOURCE(60, INTAKE_POWER),
        RETRACTED(0, -0.3);
        

        private final double angle;
        private final double power;

        private IntakeState(double angle, double power) {
            this.angle = angle;
            this.power = power;
        }

        public double getAngle() {
            return angle;
        }

        public double getPower() {
            return power;
        }
    }

    private static Intake instance;
    private static IntakeState goalState;

    public static synchronized Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    private Intake() {
        super(CONTROLLER, PIVT_MOTOR);

        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));

        configController();
        configTriggers();
        initShuffleboard();

        setDefaultCommand(setState(IntakeState.RETRACTED));
    }

    @Override
    protected void configMotors() {
        PIVT_MOTOR.setInverted(PIVT_MOTOR_INVERT);
        PIVT_MOTOR.setCurrentLimit(PIVT_CURRENT_LIMIT);
        PIVT_MOTOR.setUnitConversionFactor(PIVT_GEAR_RATIO);
        PIVT_MOTOR.setStatusFrames(PIVT_STATUS_FRAME);
        PIVT_MOTOR.setNeutralMode(PIVT_NEUTRAL_MODE);

        ROLLER_MOTOR.setInverted(ROLLER_MOTOR_INVERT);
        ROLLER_MOTOR.setNeutralMode(ROLLER_NEUTRAL_MODE);
        ROLLER_MOTOR.setUnitConversionFactor(ROLLER_GEAR_RATIO);
        ROLLER_MOTOR.setStatusFrames(ROLLER_STATUS_FRAME);
        ROLLER_MOTOR.setNeutralMode(ROLLER_NEUTRAL_MODE);
        ROLLER_MOTOR.enableVoltageCompensation(ROLLER_VOLT_COMP);
    }

    public void configController(){
        CONTROLLER.setConstraints(POSITION_MIN, POSITION_MAX);
        CONTROLLER.setTolerance(POSITION_TOLERANCE);
        this.setSafetyThresh(2);
    }

    public void configTriggers(){
        new Trigger(()-> Shooter.getInstance().hasObjectPresent())
        .and(()-> Hopper.getInstance().hasObjectPresent())
        .onTrue(setState(IntakeState.RETRACTED));
    }

    public Command runRollers(double power) {
        return runOnce(()-> ROLLER_MOTOR.set(power));
    }

    public Command setState(IntakeState state) {
        goalState = state;
        return sequence(
            pivotTo(state.getAngle()),
            runRollers(state.getPower())
        );
    }

    public IntakeState getGoalState() {
        return goalState;
    }

    public boolean goalStateIs(IntakeState state) {
        return goalState == state;
    }

}
package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import common.core.controllers.TrapController;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends PivotTemplate{

    public enum Setpoint {
        GROUND(200),
        SOURCE(60),
        NEUTRAL(0);
        

        public final double angle;
        private Setpoint(double angle) {
            this.angle = angle;
        }
    }

    private static Intake instance;

    public static synchronized Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    private Intake() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);

        setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
        setTolerance(ANGLE_TOLERANCE);
        setConstraints(MIN_SETPOINT, MAX_SETPOINT);
        setSafetyThresh(5);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        PIVOT_MOTOR.setInverted(false);
        PIVOT_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
        PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
        PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);

        RIGHT_ROLLER_MOTOR.setInverted(false);
        LEFT_ROLLER_MOTOR.setInverted(true);
        RIGHT_ROLLER_MOTOR.enableVoltageCompensation(VOLT_COMP);
        LEFT_ROLLER_MOTOR.follow(RIGHT_ROLLER_MOTOR);
        RIGHT_ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
        RIGHT_ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        RIGHT_ROLLER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    public Command retract() {
        return sequence(
            pivotTo(5),
            waitUntil(()-> atSetpoint()).withTimeout(1.5),
            runPivot(-0.2),
            waitSeconds(0.1),
            runPivot(0),
            reset(0)
        );
    }

    public Command pivotTo (Setpoint setpoint) {
        return pivotTo(setpoint.angle);
    }

    public Command runIntakeRollers() {
        return runRollers(INTAKE_POWER);
    }

    public Command runOuttakeRollers() {
        return runRollers(OUTTAKE_POWER);
    }

    public Command stopRollers() {
        return runRollers(0);
    }

    public Command runRollers(double power) {
        return runOnce(()-> RIGHT_ROLLER_MOTOR.set(power));
    }

}

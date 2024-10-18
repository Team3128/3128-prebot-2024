package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import common.core.controllers.TrapController;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team3128.Constants.IntakeConstants;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends PivotTemplate{

    public enum Setpoint {
        GROUND(133),
        OUTTAKE(90),
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
        setSafetyThresh(1.5);
        initShuffleboard();
        reset(0);

        //TODO: remove once done testing
        // this.setSafetyThresh(1000);

    }

    @Override
    protected void configMotors() {
        PIVOT_MOTOR.setInverted(true);
        PIVOT_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
        PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
        PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);

        ROLLER_MOTOR.setInverted(false);
        ROLLER_MOTOR.enableVoltageCompensation(VOLT_COMP);
        ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
        ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        
        ROLLER_MOTOR2.setInverted(false);
        ROLLER_MOTOR2.enableVoltageCompensation(VOLT_COMP);
        ROLLER_MOTOR2.setNeutralMode(Neutral.COAST);
        ROLLER_MOTOR2.setCurrentLimit(CURRENT_LIMIT);
        ROLLER_MOTOR2.follow(ROLLER_MOTOR);
    }

    public Command retract() {
        return sequence(
            pivotTo(Setpoint.NEUTRAL),
            runOuttakeRollers(),
            waitUntil(()-> atSetpoint()).withTimeout(1.5)
        ).andThen(runRollers(0));
    }

    public Command outtake(){
        return sequence(
            pivotTo(Setpoint.OUTTAKE),
            runRollers(-1)
        );

    }

    @Override
    public void initShuffleboard(){
        super.initShuffleboard();
        NAR_Shuffleboard.addData(getName(), "Current", ()-> ROLLER_MOTOR.getStallCurrent(), 4, 0);
    }

    @Override
    public double getMeasurement() {
        return motors[0].getPosition();
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
        return runOnce(()-> ROLLER_MOTOR.set(power));
    }

    public void setVoltage(double volts){
        PIVOT_MOTOR.setVolts(volts);
    }

}
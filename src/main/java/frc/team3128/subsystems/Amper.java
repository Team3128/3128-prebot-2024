package frc.team3128.subsystems;

import static frc.team3128.Constants.AmperConstants.*;

import common.core.controllers.TrapController;
import common.core.subsystems.ElevatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Amper extends ElevatorTemplate {
    
    public enum Setpoint {
        EXTENDED(20),
        RETRACTED(0);

        private double setpoint;
        private Setpoint(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    private static Amper instance;

    public static synchronized Amper getInstance() {
        if (instance == null)
            instance = new Amper();
        return instance;
    }

    private Amper() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), ELEV_MOTOR);

        // TODO: figure out kG function
        // setkG_Function(());
        setTolerance(POSITION_TOLERANCE);
        setConstraints(MIN_SETPOINT, MAX_SETPOINT);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        // TODO: figure unit conversion out
        ELEV_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
        ELEV_MOTOR.setCurrentLimit(CURRENT_LIMIT);

        ELEV_MOTOR.setNeutralMode(Neutral.BRAKE);
        ROLLER_MOTOR.setNeutralMode(Neutral.COAST);

        ELEV_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        ROLLER_MOTOR.setDefaultStatusFrames();
    }

    public Command moveElevator(Setpoint setpoint) {
        return moveElevator(setpoint.setpoint);
    }

    public Command runRollers() {
        return runRollers(ROLLER_POWER);
    }

    public Command stopRollers(){
        return runRollers(0);
    }

    public Command runRollers(double power) {
        return runOnce(() -> ROLLER_MOTOR.set(power));
    }  

    public Command extend() {
        return sequence(
            moveElevator(Setpoint.EXTENDED),
            runRollers()
        );
    }

    public Command retract() {
        return sequence(
            moveElevator(Setpoint.RETRACTED),
            stopRollers()
        );
    }

}

package frc.team3128.subsystems;

import common.core.controllers.TrapController;
import common.core.subsystems.ElevatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.team3128.Constants.ClimberConstants.*;

public class Climber extends ElevatorTemplate {

    public enum Setpoint {
        EXTENDED(30),
        RETRACTED(0);

        private double setpoint;
        private Setpoint(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    private static Climber instance;

    public static synchronized Climber getInstance() {
        if (instance == null)
            instance = new Climber();
        return instance;
    }

    private Climber() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), CLIMB_MOTOR);

        // TODO: figure out kG function
        // setkG_Function(());
        setTolerance(POSITION_TOLERANCE);
        setConstraints(MIN_SETPOINT, MAX_SETPOINT);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        // TODO: figure unit conversion out
        CLIMB_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
        CLIMB_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        CLIMB_MOTOR.setNeutralMode(Neutral.BRAKE);
        CLIMB_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
    }

    public Command moveElevator(Setpoint setpoint) {
        return moveElevator(setpoint.setpoint);
    }

}

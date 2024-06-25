package frc.team3128.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.team3128.Constants.AmperConstants.*;
import static frc.team3128.Constants.IntakeConstants.AMP_POWER;

import common.core.controllers.TrapController;
import common.core.subsystems.ElevatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;

public class Amper extends ElevatorTemplate {
    
    public enum Setpoint {
        AMP(20),
        RETRACTED(0);

        private double angle;
        private Setpoint(double angle) {
            this.angle = angle;
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
        setConstraints(0, 25);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        // TODO: figure unit conversion out
        ELEV_MOTOR.setUnitConversionFactor(ELEV_MOTOR_ID);
        ELEV_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        ELEV_MOTOR.setNeutralMode(Neutral.BRAKE);

        ROLLER_MOTOR.setNeutralMode(Neutral.COAST);

        ELEV_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        ROLLER_MOTOR.setDefaultStatusFrames();
    }

    public Command moveElevator(Setpoint setpoint) {
        return moveElevator(setpoint.angle);
    }

    public Command retract() {
        return sequence(
            moveElevator(Setpoint.RETRACTED),
            runOnce(() -> ROLLER_MOTOR.set(0))
        );
    }

    public Command extend() {
        return sequence(
            moveElevator(Setpoint.AMP),
            runRollers(AMP_POWER)
        );
    }

    public Command runRollers(double power) {
        return runOnce(()-> ROLLER_MOTOR.set(power));
    }
}

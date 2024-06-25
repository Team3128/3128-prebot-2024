package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import common.core.controllers.TrapController;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.PivotTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake {

    public enum Setpoint {
        EXTENDED(200),
        SOURCE(60),
        AMP(90);
        

        public final double angle;
        private Setpoint(double angle) {
            this.angle = angle;
        }
    }

    public class IntakePivot extends PivotTemplate {
        private IntakePivot() {
            super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);
            setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
            setTolerance(ANGLE_TOLERANCE);
            setConstraints(POSITION_MINIMUM, POSITION_MAXIMUM);
            setSafetyThresh(5);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            PIVOT_MOTOR.setInverted(true);
            PIVOT_MOTOR.setUnitConversionFactor(360 * GEAR_RATIO);
            PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
            PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        }

        @Override
        public void useOutput(double output, double setpoint) {
            PIVOT_MOTOR.setVolts(MathUtil.clamp(output, -12, 12));
        }

        public Command pivotTo(DoubleSupplier setpoint) {
            return runOnce(()-> startPID(setpoint.getAsDouble()));
        }

        public Command pivotTo(double setpoint) {
            return pivotTo(()->setpoint);
        }
    }

    public class IntakeRollers extends ManipulatorTemplate {

        private DigitalInput limitSwitch;
        
        private IntakeRollers() {
            super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, RIGHT_ROLLER_MOTOR);
            limitSwitch = new DigitalInput(9);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            RIGHT_ROLLER_MOTOR.setInverted(false);
            LEFT_ROLLER_MOTOR.setInverted(true);
            RIGHT_ROLLER_MOTOR.enableVoltageCompensation(9);
            LEFT_ROLLER_MOTOR.follow(RIGHT_ROLLER_MOTOR);
            RIGHT_ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
            RIGHT_ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            RIGHT_ROLLER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        @Override
        public boolean hasObjectPresent() {
            return !limitSwitch.get();
        }
    }

    private static Intake instance;
    public IntakePivot intakePivot;
    public IntakeRollers intakeRollers;

    public boolean isRetracting = false;

    public static synchronized Intake getInstance(){
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    private Intake() {
        intakePivot = new IntakePivot();
        intakeRollers = new IntakeRollers();
    }
}

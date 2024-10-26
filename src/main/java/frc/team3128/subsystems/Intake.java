package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import common.core.controllers.Controller;
import common.core.controllers.TrapController;
import common.core.subsystems.PivotTemplate;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends SubsystemBase {
    private final NAR_CANSpark pivotMotor = new NAR_CANSpark(PIVOT_MOTOR_ID);
    private final NAR_TalonFX rollerMotor1 = new NAR_TalonFX(ROLLER_MOTOR_ID1);
    private final NAR_TalonFX rollerMotor2 = new NAR_TalonFX(ROLLER_MOTOR_ID2);

    public class IntakePivot extends PivotTemplate {
        private IntakePivot() {
            super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), pivotMotor);
            setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
            setTolerance(ANGLE_TOLERANCE);
            setConstraints(MIN_SETPOINT, MAX_SETPOINT);
            setSafetyThresh(1.5);
            reset(0);
            setName("IntakePivot");
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            pivotMotor.setInverted(true);
            pivotMotor.setUnitConversionFactor(UNIT_CONV_FACTOR);
            pivotMotor.setNeutralMode(Neutral.BRAKE);
            pivotMotor.setStatusFrames(SparkMaxConfig.POSITION);
        }
    }

    public class IntakeRollers extends ShooterTemplate {
        private IntakeRollers() {
            super(new Controller(ROLLER_PID, Controller.Type.VELOCITY), rollerMotor1, rollerMotor2);
            setTolerance(ROLLER_TOLERANCE);
            setConstraints(ROLLER_MIN_RPM, ROLLER_MAX_RPM);
            setName("IntakeRollers");
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            rollerMotor1.setVelocityStatusFrames();
            rollerMotor1.setInverted(false);
            rollerMotor1.setNeutralMode(Neutral.COAST);
            rollerMotor1.setCurrentLimit(CURRENT_LIMIT);

            rollerMotor2.setVelocityStatusFrames();
            rollerMotor2.setInverted(true);
            rollerMotor2.setNeutralMode(Neutral.COAST);
            rollerMotor2.setCurrentLimit(CURRENT_LIMIT);
        }
    }

    public IntakePivot pivot;
    public IntakeRollers rollers;
    
    private Intake() {
        pivot = new IntakePivot();
        rollers = new IntakeRollers();

        NAR_Shuffleboard.addData(getName(), "GROUND", ()-> isState(IntakeState.GROUND), 0, 0);
        NAR_Shuffleboard.addData(getName(), "OUTTAKE", ()-> isState(IntakeState.OUTTAKE), 1, 0);
        NAR_Shuffleboard.addData(getName(), "NEUTRAL", ()-> isState(IntakeState.NEUTRAL), 2, 0);
        NAR_Shuffleboard.addData(getName(), "At Setpoint", ()-> atSetpoint(), 0, 1);

    }

    public enum IntakeState {
        GROUND(133, 4500),
        OUTTAKE(90, -4500),
        NEUTRAL(0, 0);
        
        private double pivotSetpoint;
        private double rollerSetpoint;
        private boolean disableOnCompletion;

        private IntakeState(double pivotSetpoint, double rollerSetpoint) {
            this.pivotSetpoint = pivotSetpoint;
            this.rollerSetpoint = rollerSetpoint;
        }

        private IntakeState(double pivotSetpoint, double rollerSetpoint, boolean disableOnCompletion) {
            this.pivotSetpoint = pivotSetpoint;
            this.rollerSetpoint = rollerSetpoint;
            this.disableOnCompletion = disableOnCompletion;
        }

        public double getIntakePivotSetpoint() {
            return pivotSetpoint;
        }

        public double getIntakeRollerSetpoint() {
            return rollerSetpoint;
        }

        public boolean getDisableOnCompletion() {
            return disableOnCompletion;
        }
    }

    private static Intake instance;


    public static synchronized Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    public Command setState(IntakeState state) {
        return setState(state, 0);
    }

    public Command setState(IntakeState state, double delay) {
        return sequence(
            rollers.shoot(state.getIntakeRollerSetpoint()),
            pivot.pivotTo(state.getIntakePivotSetpoint()),
            waitSeconds(delay),
            waitUntil(()-> atSetpoint()),
            either(disable(), waitUntil(()-> pivot.atSetpoint()), () -> state.disableOnCompletion)
        );
    }

    public Command disable() {
        return sequence(
            runOnce(()-> pivot.disable()),
            runOnce(()-> rollers.disable())
        ).ignoringDisable(true);
    }

    public boolean isState(IntakeState state) {
        return state.getIntakePivotSetpoint() == pivot.getSetpoint()
        && state.getIntakeRollerSetpoint() == rollers.getSetpoint()
        && atSetpoint();
    }

    public boolean atSetpoint() {
        return pivot.atSetpoint() && rollers.atSetpoint();
    }

    public  Command reset() {
        return sequence(
            disable(),
            pivot.reset(0)
        );
    }
}
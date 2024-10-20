package frc.team3128.subsystems;

import static frc.team3128.Constants.IntakeConstants.*;

import common.core.controllers.Controller;
import common.core.controllers.TrapController;
import common.core.subsystems.PivotTemplate;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends SubsystemBase {

    public class IntakePivot extends PivotTemplate {
        private IntakePivot() {
            super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);
            setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));
            setTolerance(ANGLE_TOLERANCE);
            setConstraints(MIN_SETPOINT, MAX_SETPOINT);
            setSafetyThresh(1.5);
            reset(0);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            PIVOT_MOTOR.setInverted(true);
            PIVOT_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
            PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
            PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        }
    }

    public class IntakeRollers extends ShooterTemplate {
        private IntakeRollers() {
            super(new Controller(ROLLER_PID, Controller.Type.VELOCITY), ROLLER_MOTOR1, ROLLER_MOTOR2);
            setTolerance(ROLLER_TOLERANCE);
            setConstraints(ROLLER_MIN_RPM, ROLLER_MAX_RPM);
            initShuffleboard();
        }

        @Override
        protected void configMotors() {
            ROLLER_MOTOR1.setVelocityStatusFrames();
            ROLLER_MOTOR1.setInverted(false);
            ROLLER_MOTOR1.setNeutralMode(Neutral.COAST);
            ROLLER_MOTOR1.setCurrentLimit(CURRENT_LIMIT);

            ROLLER_MOTOR2.setVelocityStatusFrames();
            ROLLER_MOTOR2.setInverted(true);
            ROLLER_MOTOR2.setNeutralMode(Neutral.COAST);
            ROLLER_MOTOR2.setCurrentLimit(CURRENT_LIMIT);
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
            setState(IntakeState.NEUTRAL),
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

    // public Command retract() {
    //     return sequence(
    //         pivotTo(IntakeState.NEUTRAL),
    //         runOuttakeRollers(),
    //         waitUntil(()-> atSetpoint()).withTimeout(1.5)
    //     ).andThen(runRollers(0));
    // }

    // public Command outtake(){
    //     return sequence(
    //         pivotTo(IntakeState.OUTTAKE),
    //         runRollers(-1)
    //     );

    // }

    // @Override
    // public void initShuffleboard(){
    //     super.initShuffleboard();
    //     NAR_Shuffleboard.addData(getName(), "Current", ()-> ROLLER_MOTOR.getStallCurrent(), 4, 0);
    // }

    // @Override
    // public double getMeasurement() {
    //     return motors[0].getPosition();
    // }

    // public Command pivotTo (IntakeState setpoint) {
    //     return pivotTo(state.angle);
    // }

    // public Command runIntakeRollers() {
    //     return runRollers(INTAKE_POWER);
    // }

    // public Command runOuttakeRollers() {
    //     return runRollers(OUTTAKE_POWER);
    // }

    // public Command stopRollers() {
    //     return runRollers(0);
    // }

    // public void setVoltage(double volts){
    //     PIVOT_MOTOR.setVolts(volts);
    // }
}
// public class Intake extends PivotTemplate {

//     public enum Setpoint {
//         GROUND(133),
//         OUTTAKE(90),
//         NEUTRAL(0);
        

//         public final double angle;
//         private Setpoint(double angle) {
//             this.angle = angle;
//         }
//     }

//     private static Intake instance;

//     public static synchronized Intake getInstance() {
//         if (instance == null)
//             instance = new Intake();
//         return instance;
//     }

//     private Intake() {
//         super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), PIVOT_MOTOR);

//         setkG_Function(()-> Math.cos(Units.degreesToRadians(getSetpoint())));

//         setTolerance(ANGLE_TOLERANCE);
//         setConstraints(MIN_SETPOINT, MAX_SETPOINT);
//         setSafetyThresh(1.5);
//         initShuffleboard();
//         reset(0);

//         //TODO: remove once done testing
//         // this.setSafetyThresh(1000);

//     }

//     @Override
//     protected void configMotors() {
//         PIVOT_MOTOR.setInverted(true);
//         PIVOT_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
//         PIVOT_MOTOR.setNeutralMode(Neutral.COAST);
//         PIVOT_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);

//         // ROLLER_MOTOR.setInverted(false);
//         // ROLLER_MOTOR.enableVoltageCompensation(VOLT_COMP);
//         // ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
//         // ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
//         // ROLLER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
//     }

//     public Command retract() {
//         return sequence(
//             pivotTo(Setpoint.NEUTRAL),
//             runOuttakeRollers(),
//             waitUntil(()-> atSetpoint()).withTimeout(1.5)
//         ).andThen(runRollers(0));
//     }

//     public Command outtake(){
//         return sequence(
//             pivotTo(Setpoint.OUTTAKE),
//             runRollers(-1)
//         );

//     }

//     @Override
//     public void initShuffleboard(){
//         super.initShuffleboard();
//         NAR_Shuffleboard.addData(getName(), "Current", ()-> ROLLER_MOTOR.getStallCurrent(), 4, 0);
//     }

//     @Override
//     public double getMeasurement() {
//         return motors[0].getPosition();
//     }

//     public Command pivotTo (Setpoint setpoint) {
//         return pivotTo(setpoint.angle);
//     }

//     public Command runIntakeRollers() {
//         return runRollers(INTAKE_POWER);
//     }

//     public Command runOuttakeRollers() {
//         return runRollers(OUTTAKE_POWER);
//     }

//     public Command stopRollers() {
//         return runRollers(0);
//     }

//     public Command runRollers(double power) {
//         return runOnce(()-> ROLLER_MOTOR.set(power));
//     }

//     public void setVoltage(double volts){
//         PIVOT_MOTOR.setVolts(volts);
//     }

// }
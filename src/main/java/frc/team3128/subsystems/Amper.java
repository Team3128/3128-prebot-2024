package frc.team3128.subsystems;

import static frc.team3128.Constants.AmperConstants.*;

import common.core.controllers.Controller;
import common.core.controllers.TrapController;
import common.core.subsystems.ElevatorTemplate;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Amper extends SubsystemBase{
    
    public class AmpElevator extends ElevatorTemplate {
    
        private AmpElevator() {
            super(new TrapController(ELEVATOR_PID, TRAP_CONSTRAINTS), ELEV_MOTOR);
            setTolerance(POSITION_TOLERANCE);
            setConstraints(MIN_SETPOINT, MAX_SETPOINT);    
        }
    
        @Override
        protected void configMotors(){
            ELEV_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
            ELEV_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            ELEV_MOTOR.setInverted(true);
            ELEV_MOTOR.setNeutralMode(Neutral.BRAKE);
            ELEV_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        }
    
    }
    
    public class AmpManipulator extends ShooterTemplate {
    
        private AmpManipulator() {
            super(new Controller(ROLLER_PID, Controller.Type.VELOCITY), ROLLER_MOTOR);
            setTolerance(ROLLER_TOLERANCE);
            setConstraints(ROLLER_MIN_RPM, ROLLER_MAX_RPM);
        }
    
        @Override
        protected void configMotors(){
            ROLLER_MOTOR.setVelocityStatusFrames();
            ROLLER_MOTOR.setInverted(true);
            ROLLER_MOTOR.setNeutralMode(Neutral.COAST);
            ROLLER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        }
    }
    

    public enum AmpState {
        EXTENDED(21.25, 5500),
        PRIMED(21.25*0.7, 5500),
        IDLE(0, 0, true),
        RETRACTED(0, 5500, true);

        private double elevatorSetpoint;
        private double rollerSetpoint;
        private boolean disableOnCompletion;

        private AmpState(double elevatorSetpoint, double rollerSetpoint, boolean disableOnCompletion){
            this.elevatorSetpoint = elevatorSetpoint;
            this.rollerSetpoint = rollerSetpoint;
            this.disableOnCompletion = disableOnCompletion;
        }
       
        private AmpState(double elevatorSetpoint, double rollerSetpoint) {
            this.elevatorSetpoint = elevatorSetpoint;
            this.rollerSetpoint = rollerSetpoint;
        }

        public double getElevatorSetpoint() {
            return elevatorSetpoint;
        }

        public double getRollerSetpoint() {
            return rollerSetpoint;
        }

        public boolean disableOnCompletion() {
            return disableOnCompletion;
        }
    }

    private static Amper instance;

    public static AmpElevator elevator;
    public static AmpManipulator manipulator;

    private static AmpState state;

    public static synchronized Amper getInstance() {
        if (instance == null)
            instance = new Amper();
        return instance;
    }

    private Amper() {
        elevator = new AmpElevator();
        manipulator = new AmpManipulator();

        setDefaultCommand(setState(AmpState.RETRACTED));
    }

    public Command setState(AmpState state) {
        return setState(state, 0);
    }

    public Command setState(AmpState state, double delay) {
        Amper.state = state;
        return sequence(
            manipulator.shoot(state.getRollerSetpoint()),
            elevator.moveElevator(state.getElevatorSetpoint()),
            waitSeconds(delay),
            Commands.either(disable(), waitUntil(()-> atSetpoint()), ()-> state.disableOnCompletion())
        );
    }

    public AmpState getState() {
        return state;
    }

    public boolean isState(AmpState state) {
        return Amper.state == state;
    }

    public boolean atSetpoint() {
        return elevator.atSetpoint() && manipulator.atSetpoint();
    }

    public  Command reset() {
        return sequence(
            disable(),
            elevator.reset(0)
        );
    
    }

    public Command disable() {
        return sequence(
            Commands.runOnce(()-> elevator.disable()),
            Commands.runOnce(()-> manipulator.disable())
        ).ignoringDisable(true);
    }
}
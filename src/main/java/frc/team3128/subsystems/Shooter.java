package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.ShooterTemplate;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.subsystems.Amper.AmpState;
import frc.team3128.subsystems.Hopper.HopperState;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.Flags.*;
import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends ShooterTemplate {

    public enum ShooterState{
        IDLE(0, 0, null),
        SHOOT(SHOOTER_RPM, KICK_POWER, Swerve.getInstance()::getTurnAngle), //set to null if not working
        EDGE_FEED(EDGE_FEED_RPM, KICK_POWER, ()-> EDGE_FEED_ANGLE),
        MIDDLE_FEED(MIDDLE_FEED_RPM, KICK_POWER, ()-> MIDDLE_FEED_ANGLE),
        PRIMED(AMP_RPM, 0, ()-> AMP_ANGLE),
        AMP(AMP_RPM, KICK_POWER, ()-> AMP_ANGLE),
        RECIEVE(0, KICK_POWER, ()-> AMP_ANGLE);

        private final double rmp; // can change to power if you want
        private final double kickPower;
        private DoubleSupplier robotAngle;

        private ShooterState(double rmp, double kickPower, DoubleSupplier robotAngle){
            this.rmp = rmp;
            this.kickPower = kickPower;
            this.robotAngle = robotAngle;
        }

        public double getRPM(){
            return rmp;
        }

        public double getKickPower(){
            return kickPower;
        }

        public DoubleSupplier getRobotAngle(){
            return robotAngle;
        }
    }

    private static Shooter instance;
    private static ShooterState goalState;

    public static synchronized Shooter getInstance(){
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    private Shooter() {
        super(new Controller(PIDConstants, Type.VELOCITY), SHTR_MOTOR);

        configMotors();
        configController();
        initShuffleboard();

        setDefaultCommand(setState(ShooterState.IDLE));
    }

    @Override
    protected void configMotors() {
        SHTR_MOTOR.setInverted(SHTR_MOTOR_INVERT);
        SHTR_MOTOR.setCurrentLimit(SHTR_CURRENT_LIMIT);
        SHTR_MOTOR.setUnitConversionFactor(SHTR_GEAR_RATIO);
        SHTR_MOTOR.setStatusFrames(SHTR_STATUS_FRAME);
        SHTR_MOTOR.setNeutralMode(SHTR_NEUTRAL_MODE);

        KICK_MOTOR.setInverted(KICK_MOTOR_INVERT);
        KICK_MOTOR.setCurrentLimit(KICK_CURRENT_LIMIT);
        KICK_MOTOR.setNeutralMode(KICK_NEUTRAL_MODE);
        KICK_MOTOR.setUnitConversionFactor(KICK_GEAR_RATIO);
        KICK_MOTOR.setStatusFrames(KICK_STATUS_FRAME);
    }

    public void configController(){
        getController().setConstraints(VELOCITY_MIN, VELOCITY_MAX);
        getController().setTolerance(TOLERANCE);
        setSafetyThresh(2);
    }

    public void configTriggers(){
        new Trigger(shooterHasNote)
        .and(()-> Amper.goalStateIs(AmpState.PRIMED))
        .onTrue(setState(ShooterState.PRIMED));

        new Trigger(shooterHasNote)
        .and(()-> Amper.goalStateIs(AmpState.AMP))
        .and(()-> Amper.getInstance().atSetpoint())
        .onTrue(setState(ShooterState.AMP));

        new Trigger(hasNoNotes)
        .onTrue(setState(ShooterState.IDLE));

        new Trigger(()-> goalStateIs(ShooterState.IDLE))
        .and(()-> Hopper.goalStateIs(HopperState.ADVANCE))
        .onTrue(setState(ShooterState.RECIEVE));

        new Trigger(shooterHasNote)
        .and(()-> goalStateIs(ShooterState.RECIEVE))
        .onTrue(setState(ShooterState.IDLE));
    }

    public boolean hasObjectPresent(){
        return !SHTR_SENSOR.get();
    }

    public Command runKickMotor(double power) {
        return runOnce(() -> KICK_MOTOR.set(power));
    }

    public Command setState(ShooterState state){
        goalState = state;
        return parallel(
            sequence(
                shoot(state.getRPM()),
                waitUntil(()-> atSetpoint())
            ),
            // comment this out if you dont want auto rotate when state change
            Swerve.getInstance().turnInPlace(state.getRobotAngle(), 2)
        ).andThen(runKickMotor(state.getKickPower()));
    }

    public static ShooterState getGoalState(){
        return goalState;
    }

    public static boolean goalStateIs(ShooterState state){
        return goalState == state;
    }
}

package frc.team3128.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;

import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.subsystems.Amper.AmpState;
import frc.team3128.subsystems.Hopper.HopperState;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Shooter.ShooterState;

public class SubsystemManager {
    
    public enum RobotState {
        //AMP, HOPPER, INTAKE, SHOOTER
        FULL_IDLE(AmpState.IDLE, HopperState.IDLE, IntakeState.NEUTRAL, ShooterState.IDLE),
        OUTTAKE(AmpState.IDLE, HopperState.REVERSE, IntakeState.OUTTAKE, ShooterState.IDLE),
        INTAKE_SECOND(AmpState.IDLE, HopperState.HOPPER_FORWARD, IntakeState.GROUND, ShooterState.IDLE, ()-> Hopper.hopperHasObjectPresent(), RobotState.FULL_IDLE),
        INTAKE_FIRST(AmpState.IDLE, HopperState.FULL_FORWARD, IntakeState.GROUND, ShooterState.IDLE, ()-> Hopper.kickerHasObjectPresent(), RobotState.INTAKE_SECOND),
        SHOOTING_RAMP(AmpState.IDLE, HopperState.KICKER_PULL_BACK, IntakeState.NEUTRAL, ShooterState.SHOOT),
        AMPING_RAMP(AmpState.PRIMED, HopperState.KICKER_PULL_BACK, IntakeState.NEUTRAL, ShooterState.AMP),
        AMP_SECOND(AmpState.EXTENDED, HopperState.FULL_FORWARD, IntakeState.NEUTRAL, ShooterState.AMP, ()-> Hopper.hasNoObjects(), RobotState.FULL_IDLE),
        AMP_FIRST(AmpState.EXTENDED, HopperState.KICKER_FORWARD, IntakeState.NEUTRAL, ShooterState.AMP, ()-> !Hopper.kickerHasObjectPresent(), RobotState.AMP_SECOND),
        SHOOT_SECOND(AmpState.IDLE, HopperState.FULL_FORWARD, IntakeState.NEUTRAL, ShooterState.SHOOT, ()-> Hopper.hasNoObjects(), RobotState.FULL_IDLE),
        SHOOT_FIRST(AmpState.IDLE, HopperState.KICKER_FORWARD, IntakeState.NEUTRAL, ShooterState.SHOOT, ()-> !Hopper.kickerHasObjectPresent(), RobotState.SHOOT_SECOND),;

        private AmpState ampState;
        private HopperState hopperState;
        private IntakeState intakeState;
        private ShooterState shooterState;
        private BooleanSupplier condition;
        private RobotState nextState;

        private RobotState(AmpState ampState, HopperState hopperState, IntakeState intakeState, ShooterState shooterState) {
            this(ampState, hopperState, intakeState, shooterState, ()-> true, null);
        }

        private RobotState(AmpState ampState, HopperState hopperState, IntakeState intakeState, ShooterState shooterState, BooleanSupplier condition, RobotState nextState) {
            this.ampState = ampState;
            this.hopperState = hopperState;
            this.intakeState = intakeState;
            this.shooterState = shooterState;
            this.condition = condition;
            this.nextState = nextState;
        }

        

        public AmpState getAmpState() {
            return ampState;
        }

        public HopperState getHopperState() {
            return hopperState;
        }

        public IntakeState getIntakeState() {
            return intakeState;
        }

        public ShooterState getShooterState() {
            return shooterState;
        }

        public BooleanSupplier getCondition() {
            return condition;
        }

        public RobotState getNextState() {
            return nextState;
        }
    }

    private static SubsystemManager instance;

    public static synchronized SubsystemManager getInstance() {
        if (instance == null)
            instance = new SubsystemManager();
        return instance;
    }

    private Amper amper;
    private Hopper hopper;
    private Intake intake;
    private Shooter shooter;

    private SubsystemManager() {
        amper = Amper.getInstance();
        hopper = Hopper.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();

        // setName("Robot");
        initShuffleboard();
    }

    public Command setState(RobotState state, double delay) {
    
        
        return sequence(
            waitSeconds(delay),
            hopper.setState(state.getHopperState()).onlyIf(()-> HopperState.equals(state.getHopperState(), HopperState.KICKER_PULL_BACK)),
            shooter.setState(state.getShooterState()),
            amper.setState(state.getAmpState()),
            intake.setState(state.getIntakeState()),
            waitUntil(()-> shooter.atSetpoint() && amper.atSetpoint()),
            hopper.setState(state.getHopperState()),
            waitUntil(state.getCondition()),
            (state.getNextState() != null) ? setState(state.getNextState(), 0) : none()
        );
    }

    public boolean isState(RobotState state){
        return 
            amper.isState(state.getAmpState()) && 
            // hopper.isState(state.getHopperState()) && 
            intake.isState(state.getIntakeState()) && 
            shooter.isState(state.getShooterState())
        ;
    }

    public void initShuffleboard(){
        for(RobotState state : RobotState.values()){
            NAR_Shuffleboard.addData("Robot", state.toString(), ()-> isState(state));
        }
    }

}
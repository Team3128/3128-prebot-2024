package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;

import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import frc.team3128.subsystems.Amper.AmpState;
import frc.team3128.subsystems.Hopper.HopperState;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Shooter.ShooterState;

public class SubsystemManager {
    
    public enum RobotState {
        //AMP, HOPPER, INTAKE, SHOOTER
        
        IDLE(AmpState.IDLE, HopperState.IDLE, IntakeState.NEUTRAL, ShooterState.IDLE),
        INTAKING(AmpState.IDLE, HopperState.PROCESSING, IntakeState.INTAKE, ShooterState.IDLE),
        OUTTAKING(AmpState.IDLE, HopperState.OUTTAKING, IntakeState.OUTTAKE, ShooterState.IDLE),
        SHOOTING_READY(AmpState.IDLE, HopperState.IDLE, IntakeState.NEUTRAL, ShooterState.SHOOT),
        SHOOTING(AmpState.IDLE, HopperState.SHOOTING, IntakeState.NEUTRAL, ShooterState.SHOOT),
        AMPING_READY(AmpState.PRIMED, HopperState.IDLE, IntakeState.NEUTRAL, ShooterState.AMP),
        AMPING(AmpState.EXTENDED, HopperState.AMPING, IntakeState.NEUTRAL, ShooterState.AMP);

        private AmpState ampState;
        private HopperState hopperState;
        private IntakeState intakeState;
        private ShooterState shooterState;
        private RobotState nextState;

        private RobotState(AmpState ampState, HopperState hopperState, IntakeState intakeState, ShooterState shooterState) {
            this(ampState, hopperState, intakeState, shooterState, null);
        }

        private RobotState(AmpState ampState, HopperState hopperState, IntakeState intakeState, ShooterState shooterState, RobotState nextState) {
            this.ampState = ampState;
            this.hopperState = hopperState;
            this.intakeState = intakeState;
            this.shooterState = shooterState;
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
            runOnce(()->Log.info("State", state.toString())),
            hopper.setState(state.getHopperState()),
            shooter.setState(state.getShooterState()),
            amper.setState(state.getAmpState()),
            intake.setState(state.getIntakeState()),
            waitUntil(()-> shooter.atSetpoint() && amper.atSetpoint()),
            hopper.setState(state.getHopperState()),
            waitSeconds(delay),
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

package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.subsystems.Amper.AmpState;
import frc.team3128.subsystems.Hopper.HopperState;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Shooter.ShooterState;

public class SubsystemManager extends SubsystemBase{
    
    public enum RobotState {
        FULL_IDLE(AmpState.IDLE, HopperState.IDLE, IntakeState.NEUTRAL, ShooterState.IDLE),
        OUTTAKE(AmpState.IDLE, HopperState.OUTTAKE, IntakeState.OUTTAKE, ShooterState.IDLE),
        INTAKE_SECOND(AmpState.IDLE, HopperState.HOPPER_ONLY, IntakeState.GROUND, ShooterState.IDLE, waitUntil(()-> Hopper.hopperHasObjectPresent()), RobotState.FULL_IDLE),
        INTAKE_FIRST(AmpState.IDLE, HopperState.BOTH, IntakeState.GROUND, ShooterState.IDLE, waitUntil(()-> Hopper.kickerHasObjectPresent()), RobotState.INTAKE_SECOND),
        SHOOTING_RAMP(AmpState.IDLE, HopperState.KICKER_PULL_BACK, IntakeState.NEUTRAL, ShooterState.SHOOT),
        AMPING_RAMP(AmpState.PRIMED, HopperState.IDLE, IntakeState.NEUTRAL, ShooterState.AMP),
        AMP_SECOND(AmpState.EXTENDED, HopperState.BOTH, IntakeState.NEUTRAL, ShooterState.AMP, waitUntil(()-> Hopper.hasNoObjects()), RobotState.FULL_IDLE),
        AMP_FRIST(AmpState.EXTENDED, HopperState.KICKER_ONLY, IntakeState.NEUTRAL, ShooterState.AMP, waitUntil(()-> !Hopper.kickerHasObjectPresent()), RobotState.AMP_SECOND),
        SHOOT_SECOND(AmpState.IDLE, HopperState.BOTH, IntakeState.NEUTRAL, ShooterState.SHOOT, waitUntil(()-> Hopper.hasNoObjects()), RobotState.FULL_IDLE),
        SHOOT_FIRST(AmpState.IDLE, HopperState.KICKER_ONLY, IntakeState.NEUTRAL, ShooterState.SHOOT, waitUntil(()-> !Hopper.kickerHasObjectPresent()), RobotState.SHOOT_SECOND),;

        private AmpState ampState;
        private HopperState hopperState;
        private IntakeState intakeState;
        private ShooterState shooterState;
        private Command condition;
        private RobotState nextState;

        private RobotState(AmpState ampState, HopperState hopperState, IntakeState intakeState, ShooterState shooterState) {
            this(ampState, hopperState, intakeState, shooterState, none(), null);
        }

        private RobotState(AmpState ampState, HopperState hopperState, IntakeState intakeState, ShooterState shooterState, Command condition, RobotState nextState) {
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

        public Command getCondition() {
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
    }

    public Command setState(RobotState state, double delay) {
        return sequence(
            waitSeconds(delay),
            parallel(
                shooter.setState(state.getShooterState()),
                amper.setState(state.getAmpState()),
                intake.setState(state.getIntakeState()),
                waitUntil(()-> shooter.atSetpoint() && amper.atSetpoint()),
                hopper.setState(state.getHopperState())
            ),
            state.getCondition(),
            either(setState(state.getNextState(), 0.1), none(), ()-> state.getNextState() != null)
        );
    }

    public boolean isState(RobotState state){
        return 
            amper.isState(state.getAmpState()) && 
            hopper.isState(state.getHopperState()) && 
            intake.isState(state.getIntakeState()) && 
            shooter.isState(state.getShooterState())
        ;
    }

}

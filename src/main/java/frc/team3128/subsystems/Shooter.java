package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.controllers.ControllerBase;
import common.core.subsystems.ManipulatorTemplate;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.narwhaldashboard.NarwhalDashboard.State;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.subsystems.Intake.IntakeState;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase{

    public class Flywheel extends ShooterTemplate {
        private Flywheel(){
            super(new Controller(PIDConstants, Type.VELOCITY), SHOOTER_MOTOR);
            setConstraints(MIN_RPM, MAX_RPM);
            setTolerance(TOLERANCE);

            configMotors();
            initShuffleboard();
            controller.setTolerance(TOLERANCE);
        }
        protected void configMotors() {
            SHOOTER_MOTOR.setCurrentLimit(40);
            SHOOTER_MOTOR.setInverted(false);
            SHOOTER_MOTOR.setUnitConversionFactor(GEAR_RATIO);
            SHOOTER_MOTOR.setNeutralMode(Neutral.COAST);
            SHOOTER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

    }

    public enum ShooterState {
        AMP(AMP_RPM),
        SHOOT(SHOOTER_RPM),
        IDLE(0);
        
        private double shooterSetpoint;

        private ShooterState(double shooterSetpoint) {
            this.shooterSetpoint = shooterSetpoint;
        }

        public double getShooterSetpoint(){
            return shooterSetpoint;
        }
    }

    private static Shooter instance;
    private static ShooterState state;

    public static synchronized Shooter getInstance(){
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    public Flywheel flywheel;

    private Shooter() {
        flywheel = new Flywheel();
    }

    public Command setState(ShooterState state){
        return setState(state, 0);
    }

    public Command setState(ShooterState state, double delay){
        return sequence(
            flywheel.shoot(state.getShooterSetpoint()),
            waitSeconds(delay),
            waitUntil(()-> atSetpoint())
        );
    }

    public Command disable(){
        return sequence(
            runOnce(()-> flywheel.disable())
        ).ignoringDisable(true);
    }

    public boolean isState(ShooterState state) {
        return flywheel.getSetpoint() == state.getShooterSetpoint() 
        && atSetpoint();
    }

    public boolean atSetpoint() {
        return flywheel.atSetpoint();
    }
}

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
import frc.team3128.subsystems.Intake.IntakeState;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class Shooter extends ShooterTemplate {

    public class Flywheel extends ShooterTemplate {
        private Flywheel(){
            super(new Controller(PIDConstants, Type.VELOCITY), SHOOTER_MOTOR);
            setConstraints(MIN_RPM, MAX_RPM);
            setTolerance(TOLERANCE);

            rollersSensor = new DigitalInput(ROLLERS_SENSOR_ID);
            isShooting = false;

            configMotors();
            // initShuffleboard();
            controller.setTolerance(TOLERANCE);
        }
        protected void configMotors() {
            SHOOTER_MOTOR.setCurrentLimit(80);
            SHOOTER_MOTOR.setInverted(false);
            SHOOTER_MOTOR.setUnitConversionFactor(GEAR_RATIO);
            SHOOTER_MOTOR.setNeutralMode(Neutral.COAST);
            SHOOTER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

    }

    public class Kicker extends ManipulatorTemplate {
        private Kicker(){
            super(CURRENT_TEST_POWER, CURRENT_TEST_PLATEAU, CURRENT_TEST_EXPECTED_CURRENT, AMP_SHOOTER_POWER, AMP_RPM, KICK_MOTOR);
            configMotors();     
        }
        @Override
        protected void configMotors() {
            KICK_MOTOR.setCurrentLimit(80);
            
            KICK_MOTOR.setInverted(false);
    
            KICK_MOTOR.setNeutralMode(Neutral.COAST);
    
            KICK_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

    }

    public enum ShooterState {
        AMPPRIME(0, AMP_RPM),
        AMPSHOOT(KICK_POWER, AMP_RPM),
        SHOOTPRIME(0, SHOOTER_RPM),
        SHOOTSHOOT(KICK_POWER, SHOOTER_RPM),
        IDLE(0, 0),
        KICK(KICK_POWER, 0);
        
        private double kickPower;
        private double shooterSetpoint;

        private ShooterState(double kickPower, double shooterSetpoint) {
            this.kickPower = kickPower;
            this.shooterSetpoint = shooterSetpoint;
        }

        public double getKickPower(){
            return kickPower;
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

    private boolean isShooting;

    private DigitalInput rollersSensor;
    public Flywheel flywheel;
    public Kicker kicker;

    private Shooter() {
        super(new Controller(PIDConstants, Type.VELOCITY), SHOOTER_MOTOR);
        flywheel = new Flywheel();
        kicker = new Kicker();
    }

    public Command rampUpShooter(){
        return runShooter(SHOOTER_RPM);
    } 

    

    public Command runKickMotor(double power) {
        return runOnce(() -> KICK_MOTOR.set(power));
    }

    public Command stopMotors(){
        return runOnce(()-> {
            SHOOTER_MOTOR.set(0);
            KICK_MOTOR.set(0);
        });
    }

    public boolean hasObjectPresent() {
        return !rollersSensor.get();
    }

    public Command setState(ShooterState state){
        return sequence(
            shoot(state.getShooterSetpoint()),
            runKickMotor(state.getKickPower())
        );
    }

    public Command stop(){
        return sequence(
            runOnce(()-> disable()),
            runKickMotor(0)
        ).ignoringDisable(true);
    }
    public boolean isState(ShooterState state) {
        return Shooter.state == state;
    }

    @Override
    protected void configMotors() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configMotors'");
    }
}

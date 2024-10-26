package frc.team3128.subsystems;

import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import common.core.subsystems.ShooterTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase{

    private final NAR_CANSpark shooterMotor = new NAR_CANSpark(SHOOTER_MOTOR_ID);
    public class Flywheel extends ShooterTemplate {
        private Flywheel(){
            super(new Controller(PIDConstants, Type.VELOCITY), shooterMotor);
            setConstraints(MIN_RPM, MAX_RPM);
            setTolerance(TOLERANCE);

            configMotors();
            // initShuffleboard();
            controller.setTolerance(TOLERANCE);
        }
        protected void configMotors() {
            shooterMotor.setCurrentLimit(40);
            shooterMotor.setInverted(false);
            shooterMotor.setUnitConversionFactor(GEAR_RATIO);
            shooterMotor.setNeutralMode(Neutral.BRAKE);
            shooterMotor.setStatusFrames(SparkMaxConfig.VELOCITY);
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
            either(
                waitUntil(()-> atSetpoint()), 
                disable(), 
                ()->state.getShooterSetpoint() > 300
            )
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

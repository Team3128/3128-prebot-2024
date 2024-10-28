package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.HopperConstants.*;
import static frc.team3128.Constants.ShooterConstants.*;

import java.util.function.BooleanSupplier;

public class Hopper {

    private final NAR_CANSpark hopperMotor = new NAR_CANSpark(HOPPER_MOTOR_ID);
    public class HopperFloor extends ManipulatorTemplate {
        private DigitalInput frontSensor;
        
        private HopperFloor(){
            super(STALL_CURRENT, HOPPER_INTAKE_POWER, HOPPER_OUTTAKE_POWER, STALL_POWER, 0.3, hopperMotor);
            frontSensor = new DigitalInput(HOPPER_FRONT_SENSOR_ID);
            configMotors();
        }

        @Override
        protected void configMotors() {
            hopperMotor.enableVoltageCompensation(VOLT_COMP);
            hopperMotor.setCurrentLimit(CURRENT_LIMIT);
            hopperMotor.setNeutralMode(Neutral.COAST);
            hopperMotor.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        @Override
        public boolean hasObjectPresent() {
            return !frontSensor.get();
        }
    }

    private final NAR_CANSpark kickerMotor = new NAR_CANSpark(KICK_MOTOR_ID);

    public class Kicker extends ManipulatorTemplate {

        private DigitalInput rollersSensor;
        
        private Kicker(){
            super(STALL_CURRENT, KICKER_POWER, -1, 0, 0.5, kickerMotor);
            rollersSensor = new DigitalInput(ROLLERS_SENSOR_ID);
            configMotors();     
        }
        @Override
        protected void configMotors() {
            kickerMotor.setCurrentLimit(CURRENT_LIMIT);
            kickerMotor.setInverted(false);
            kickerMotor.setNeutralMode(Neutral.COAST);
            kickerMotor.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        @Override
        public boolean hasObjectPresent() {
            return !rollersSensor.get();
        }
    
    }

    public enum HopperState {
        // Step 1: Identify States and Describe
        // IDLE -> Do nothing
        // SHOOTING -> Wait for shooter, move kicker, wait for no object present in kicker, move hopper
        // AMPING -> Wait for shooter + amp postition + amp speed, move kicker, wait for no object present in kicker, move hopper
        // PROCESSING -> Move hopper and kicker, wait for object present in kicker, stop kicker, wait for object present in hopper, stop hopper
        // OUTTAKING -> Move hopper back

        // Step 2: Create Generic State that describes steps for state with the most steps
        // GENERIC // Wait for X, Set Kicker, Set Hopper, Wait for Y, Set Kicker, Set Hopper, Wait for Z, Set Hopper 

        // Step 3: Fully define state based on Generic State
        // IDLE // Wait for Nothing, Set Kicker(KICKER_IDLE_POWER), Set Hopper(HOPPER_IDLE_POWER), Wait for Nothing, Set Kicker (KICKER_IDLE_POWER), Set Hopper(HOPPER_IDLE_POWER), Wait for Nothing, Set Kicker (KICKER_IDLE_POWER), Set Hopper (HOPPER_IDLE_POWER)
        // SHOOTING // Wait for Shooter, Set Kicker(KICKER_SHOOTING_POWER), Set Hopper(HOPPER_IDLE_POWER), Wait for no object present in kicker, Set Kicker(KICKER_SHOOTING_POWER), Set Hopper(0), Wait for Nothing, Set Kicker(KICKER_SHOOTING_POWER), Set Hopper (HOPPER_INTAKE_POWER)
        // AMPING // Wait for shooter + amp postition + amp speed, Set Kicker(KICKER_POWER), Set Hopper(HOPPER_IDLE_POWER), Wait for no object present in kicker, Set Kicker(KICKER_POWER), Set Hopper(HOPPER_INTAKE_POWER), Wait for Nothing, Set Kicker(KICKER_POWER), Set Hopper (HOPPER_INTAKE_POWER)
        // PROCESSING // Wait for Nothing, Set Kicker(KICKER_POWER), Set Hopper(HOPPER_INTAKE_POWER), Wait for object present in kicker, Set Kicker (KICKER_IDLE_POWER), Set Hopper (HOPPER_INTAKE_POWER), Wait for object present in hopper, Set Kicker (KICKER_IDLE_POWER), Set Hopper (HOPPER_IDLE_POWER), repeat??
        // OUTTAKING // Wait for Nothing, Set Kicker(KICKER_IDLE_POWER), Set Hopper(HOPPER_OUTTAKE_POWER), Wait for Nothing, Set Kicker (KICKER_IDLE_POWER), Set Hopper(HOPPER_OUTTAKE_POWER), Wait for Nothing, Set Kicker (KICKER_IDLE_POWER), Set Hopper (HOPPER_OUTTAKE_POWER)
        
        IDLE(HopperState::dontWait, KICKER_IDLE_POWER, HOPPER_IDLE_POWER, HopperState::dontWait, KICKER_IDLE_POWER, HOPPER_IDLE_POWER, HopperState::dontWait, KICKER_IDLE_POWER, HOPPER_IDLE_POWER),
        SHOOTING(Shooter.getInstance()::atSetpoint, KICKER_SHOOTING_POWER, HOPPER_IDLE_POWER, Hopper::kickerNoObjectPresent, KICKER_SHOOTING_POWER, HOPPER_IDLE_POWER, HopperState::dontWait, KICKER_SHOOTING_POWER, HOPPER_INTAKE_POWER), 
        AMPING(HopperState::getAmpStateInitialWaitValues, KICKER_POWER, HOPPER_IDLE_POWER, Hopper::kickerNoObjectPresent, KICKER_POWER, HOPPER_INTAKE_POWER, HopperState::dontWait, KICKER_POWER, HOPPER_INTAKE_POWER), 
        PROCESSING(HopperState::dontWait, KICKER_POWER, HOPPER_INTAKE_POWER, Hopper::kickerHasObjectPresent, KICKER_IDLE_POWER, HOPPER_INTAKE_POWER, Hopper::hopperHasObjectPresent, KICKER_IDLE_POWER, HOPPER_IDLE_POWER), 
        OUTTAKING(HopperState::dontWait, KICKER_IDLE_POWER, HOPPER_OUTTAKE_POWER, HopperState::dontWait, KICKER_IDLE_POWER, HOPPER_OUTTAKE_POWER, HopperState::dontWait, KICKER_IDLE_POWER, HOPPER_OUTTAKE_POWER); 

        private BooleanSupplier initialWaitSupplier;
        private double initialKickerPower;
        private double initialHopperPower;
        private BooleanSupplier secondWaitSupplier;
        private double secondKickerPower;
        private double secondHopperPower;
        private BooleanSupplier finalWaitSupplier;
        private double finalKickerPower;
        private double finalHopperPower;

        private HopperState(BooleanSupplier initialWaitSupplier, double initialKickerPower, double initialHopperPower, 
                            BooleanSupplier secondWaitSupplier, double secondKickerPower, double secondHopperPower,
                            BooleanSupplier finalWaitSupplier, double finalKickerPower, double finalHopperPower) {
            this.initialWaitSupplier = initialWaitSupplier;
            this.initialKickerPower = initialKickerPower;
            this.initialHopperPower = initialHopperPower;
            this.secondWaitSupplier = secondWaitSupplier;
            this.secondKickerPower = secondKickerPower;
            this.secondHopperPower = secondHopperPower;
            this.finalWaitSupplier = finalWaitSupplier;
            this.finalKickerPower = finalKickerPower;
            this.finalHopperPower = finalHopperPower;
        }

        private static boolean dontWait() {
            return true;
        }

        private static boolean getAmpStateInitialWaitValues() {
            return Shooter.getInstance().atSetpoint() && Amper.getInstance().atSetpoint();
        }

        public Command getInitialWaitCommand() {
            return waitUntil(this.initialWaitSupplier);
        }

        public double getInitialKickerPower() {
            return initialKickerPower;
        }

        public double getInitialHopperPower() {
            return initialHopperPower;
        }

        public Command getSecondWaitCommand() {
            return waitUntil(this.secondWaitSupplier);
        }

        public double getSecondKickerPower() {
            return secondKickerPower;
        }

        public double getSecondHopperPower() {
            return secondHopperPower;
        }

        public Command getFinalWaitCommand() {
            return waitUntil(this.finalWaitSupplier);
        }

        public double getFinalKickerPower() {
            return finalKickerPower;
        }

        public double getFinalHopperPower() {
            return finalHopperPower;
        }
    }

    private static Hopper instance;

    public HopperFloor hopper;
    public Kicker kicker;

    private HopperState currentState;

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private Hopper() {
        hopper = new HopperFloor();
        kicker = new Kicker();
    }

    public Command setState(HopperState state) {
        currentState = state;
        return sequence(
            state.getInitialWaitCommand(),
            hopper.runManipulator(state.getInitialHopperPower()),
            kicker.runManipulator(state.getInitialKickerPower()),
            state.getSecondWaitCommand(),
            hopper.runManipulator(state.getSecondHopperPower()),
            kicker.runManipulator(state.getSecondKickerPower()),
            state.getFinalWaitCommand(),
            hopper.runManipulator(state.getFinalHopperPower()),
            kicker.runManipulator(state.getFinalKickerPower())
        );
    }

    public boolean isState(HopperState state){
        return currentState == state;
    }

    public Command disable() {
        return sequence(
            kicker.runManipulator(0),
            hopper.runManipulator(0)
        ).ignoringDisable(true);
    }

    public static boolean hasTwoObjects() {
        return instance.kicker.hasObjectPresent() && instance.hopper.hasObjectPresent();
    }

    public static boolean hasNoObjects() {
        return !instance.kicker.hasObjectPresent() && !instance.hopper.hasObjectPresent();
    }

    public static boolean kickerHasObjectPresent() {
        return instance.kicker.hasObjectPresent();
    }

    public static boolean kickerNoObjectPresent() {
        return !kickerHasObjectPresent();
    }

    public static boolean hopperHasObjectPresent() {
        return instance.hopper.hasObjectPresent();
    }
}

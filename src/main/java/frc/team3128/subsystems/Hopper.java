package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.HopperConstants.*;
import static frc.team3128.Constants.ShooterConstants.*;


public class Hopper {

    private final NAR_CANSpark hopperMotor = new NAR_CANSpark(HOPPER_MOTOR_ID);
    public class BackHopper extends ManipulatorTemplate {

        private DigitalInput frontSensor;
        
        private BackHopper(){
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
            super(STALL_CURRENT, KICK_POWER, -1, 0, 0.5, kickerMotor);
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
        FULL_FORWARD(KICK_POWER, HOPPER_INTAKE_POWER),
        HOPPER_FORWARD(0, HOPPER_INTAKE_POWER),
        HOPPER_BACKWARD(0, HOPPER_OUTTAKE_POWER),
        KICKER_FORWARD(KICK_POWER, 0),
        KICKER_PULL_BACK(-0.1, 0),
        REVERSE(-1, -.75),
        IDLE(0, 0);

        private double kickerStartPower;
        private Command kickerWaitCommand;
        private double kickerEndPower;

        private double hopperStartPower;
        private Command hopperWaitCommand;
        private double hopperEndPower;

        private HopperState(double kickerStartPower, Command kickerWaitCommand, double kickerEndPower, double hopperStartPower, Command hopperWaitCommand, double hopperEndPower) {
            this.kickerStartPower = kickerStartPower;
            this.kickerWaitCommand = kickerWaitCommand;
            this.kickerEndPower = kickerEndPower;
            this.hopperStartPower = hopperStartPower;
            this.hopperWaitCommand = hopperWaitCommand;
            this.hopperEndPower = hopperEndPower;
        }

        private HopperState(double kickerStartPower, double hopperStartPower) {
            this(kickerStartPower, none(), kickerStartPower, hopperStartPower, none(), hopperStartPower);
        }

        public double getKickerStartPower() {
            return kickerStartPower;
        }

        public Command getKickerWaitCommand() {
            return kickerWaitCommand;
        }

        public double getKickerEndPower() {
            return kickerEndPower;
        }

        public double getHopperStartPower() {
            return hopperStartPower;
        }

        public Command getHopperWaitCommand() {
            return hopperWaitCommand;
        }

        public double getHopperEndPower() {
            return hopperEndPower;
        }

        public static boolean equals(HopperState state1, HopperState state2) {
            return state1.getKickerStartPower() == state2.getKickerStartPower()
            && state1.getKickerEndPower() == state2.getKickerEndPower()
            && state1.getHopperStartPower() == state2.getHopperStartPower()
            && state1.getHopperEndPower() == state2.getHopperEndPower();
        }
    }


    private static Hopper instance;

    public BackHopper hopper;
    public Kicker kicker;

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private Hopper() {
        hopper = new BackHopper();
        kicker = new Kicker();
    }

    public Command setState(HopperState state) {
        return sequence(
            hopper.runManipulator(state.getHopperStartPower()),
            kicker.runManipulator(state.getKickerStartPower())
        );
    }

    public boolean isState(HopperState state){
        return (hopperMotor.getAppliedOutput() == state.getHopperStartPower() 
        || hopperMotor.getAppliedOutput() == state.getHopperEndPower())
        && (kickerMotor.getAppliedOutput() == state.getKickerStartPower()
        || kickerMotor.getAppliedOutput() == state.getKickerEndPower());
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

    public static boolean hopperHasObjectPresent() {
        return instance.hopper.hasObjectPresent();
    }

}

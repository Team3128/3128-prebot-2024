package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.HopperConstants.*;
import static frc.team3128.Constants.ShooterConstants.*;


public class Hopper {

    public class BackHopper extends ManipulatorTemplate {

        private DigitalInput frontSensor;
        
        private BackHopper(){
            super(STALL_CURRENT, HOPPER_INTAKE_POWER, HOPPER_OUTTAKE_POWER, STALL_POWER, 0.3, HOPPER_MOTOR);
            frontSensor = new DigitalInput(HOPPER_FRONT_SENSOR_ID);
            configMotors();
        }

        @Override
        protected void configMotors() {
            HOPPER_MOTOR.enableVoltageCompensation(VOLT_COMP);
            HOPPER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            HOPPER_MOTOR.setNeutralMode(Neutral.COAST);
            HOPPER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        }

        @Override
        public boolean hasObjectPresent() {
            return !frontSensor.get();
        }
    }

    public class Kicker extends ManipulatorTemplate {

        private DigitalInput rollersSensor;
        
        private Kicker(){
            super(STALL_CURRENT, KICK_POWER, -1, 0, 0.5, KICK_MOTOR);
            rollersSensor = new DigitalInput(ROLLERS_SENSOR_ID);
            configMotors();     
        }
        @Override
        protected void configMotors() {
            KICK_MOTOR.setCurrentLimit(CURRENT_LIMIT);
            KICK_MOTOR.setInverted(false);
            KICK_MOTOR.setNeutralMode(Neutral.COAST);
            KICK_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
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
        REVERSE(-1, -1),
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
        return parallel(
            sequence(
                kicker.runManipulator(state.getKickerStartPower()),
                state.getKickerWaitCommand(),
                kicker.runManipulator(state.getKickerEndPower())
            ),
            sequence(
                hopper.runManipulator(state.getHopperStartPower()),
                state.getHopperWaitCommand(),
                hopper.runManipulator(state.getHopperEndPower())
            )
        );
    }

    public boolean isState(HopperState state){
        return (HOPPER_MOTOR.getAppliedOutput() == state.getHopperStartPower() 
        || HOPPER_MOTOR.getAppliedOutput() == state.getHopperEndPower())
        && (KICK_MOTOR.getAppliedOutput() == state.getKickerStartPower()
        || KICK_MOTOR.getAppliedOutput() == state.getKickerEndPower());
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

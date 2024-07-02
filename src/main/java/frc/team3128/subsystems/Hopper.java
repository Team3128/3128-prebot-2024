package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.team3128.Constants.HopperConstants.*;

public class Hopper extends ManipulatorTemplate {

    private DigitalInput limitSwitch;

    private static Hopper instance;

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private Hopper() {
        super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, HOPPER_MOTOR);
        limitSwitch = new DigitalInput(1);
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        HOPPER_MOTOR.enableVoltageCompensation(VOLT_COMP);
        HOPPER_MOTOR.setNeutralMode(Neutral.BRAKE);
        HOPPER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        HOPPER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    @Override
    public boolean hasObjectPresent() {
        return !limitSwitch.get();
    }
}
